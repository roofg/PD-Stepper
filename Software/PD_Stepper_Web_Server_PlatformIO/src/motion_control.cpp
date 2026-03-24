#include "motion_control.h"
#include "encoder.h"
#include "pd_controller.h"
#include "pins.h"
#include "step_generator.h"
#include "telemetry_provider.h"
#include "tmc_driver.h"
#include "trajectory_buffer.h"
#include "usb_telemetry_provider.h"
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define STEP_PIN   TMC_STEP
#define DIR_PIN    TMC_DIR
#define SW1_PIN    PIN_SW1
#define VBUS_PIN   PIN_VBUS
#define DIV_RATIO  0.1189427313f

// Extern variables defined at global scope in main.cpp.
// DiagnosticsTask (Core 0) reads them at 1 Hz; loop() (Core 1) writes them.
// Declared volatile both here and at definition to prevent register caching.
extern volatile float    VBusVoltage;
extern volatile bool     PGState;
extern uint32_t          bootCount;

namespace motion {

// ---------------------------------------------------------------------------
// Inter-task shared state
// All are 32-bit aligned (or bool = 8-bit atomic on ESP32). No mutex needed
// for SPSC access patterns described in comments.
// ---------------------------------------------------------------------------

static volatile bool    s_running        = false; // set by planner, cleared by planner/control
static volatile float   g_meas_pos       = 0.0f;  // written: control; read: planner
static volatile float   g_target_pos     = 0.0f;  // written: planner; read: control (telemetry)
static volatile bool    g_fault_lag        = false;  // set: control; cleared: planner on move start
static volatile bool    g_fault_estop      = false;  // set: control; cleared: planner on move start
static volatile bool    g_fault_estop_gui  = false;  // set: serial cmd; cleared: planner on move start
static volatile bool    g_fault_brownout   = false;  // set: planner; cleared: planner on move start
static volatile uint16_t g_sg_result     = 0;      // written: planner (UART); read: control (tele)
// TMC telemetry cache — written by DiagnosticsTask (Core 0, ~1–10 Hz); read by ControlTask (Core 1).
// uint8_t reads/writes are single-instruction atomic on ESP32 LX7.
static volatile uint8_t  g_cs_actual_cache = 0;
static volatile uint8_t  g_pwm_scale_cache = 0;
// Microsteps per full step — written by setMicrosteps() from main task; read
// by both planner and control tasks. int32_t ensures atomic 32-bit read on LX7.
static volatile int32_t g_uSteps_setting = 32;

// PD gains — written via setPD() / setPhaseLeadGain() from main task (Core 0);
// read by ControlTask (Core 1). volatile ensures the compiler does not cache
// the value in a register across the core boundary on Xtensa LX7.
static volatile float g_kp = 3.0f;
static volatile float g_kd = 0.1f;
static volatile float g_kv = 0.0f;

// Brownout threshold — 70% of the configured USB-PD supply voltage.
// Written once from setup() via setConfiguredVoltage(); read by PlannerTask.
static volatile float g_brownout_threshold_v = 9.0f; // safe default (12V * 0.75)

// Active hold — when true, ControlTask runs PD toward g_hold_target instead
// of zeroing velocity. Written by PlannerTask; read by ControlTask.
static volatile bool  g_hold_active    = false;
static volatile float g_hold_target    = 0.0f;
// Track whether the driver has been enabled at least once since boot.
// The driver starts disabled (EN HIGH) and is enabled on the first move.
static volatile bool  s_driver_enabled = false;

// Hold state machine — managed by ControlTask
// CORRECTING: PD loop actively driving step pulses to reach target
// SETTLED:    position within deadband long enough; step pulses stopped
//             so TMC2209 detects standstill and drops to IHOLD
enum HoldState : uint8_t { HOLD_CORRECTING = 0, HOLD_SETTLED = 1 };
static volatile uint8_t  g_hold_state      = HOLD_CORRECTING;
static volatile uint32_t g_settle_start_ms = 0; // module-scope so it resets across hold activations

// Settle time: motor must stay inside deadband for this many ms before
// transitioning to SETTLED (prevents rapid flapping from encoder noise).
static constexpr uint32_t HOLD_SETTLE_MS = 500;

static QueueHandle_t     s_motionQueue   = nullptr;
static QueueHandle_t     s_teleQueue     = nullptr; // ControlTask → TelemetryTask
static TaskHandle_t      s_plannerHandle = nullptr;
static TaskHandle_t      s_controlHandle = nullptr;
static TaskHandle_t      s_teleHandle    = nullptr;
static TaskHandle_t      s_diagHandle    = nullptr;
static TelemetryProvider *s_telemetry    = nullptr;

// ---------------------------------------------------------------------------
// Trajectory Planner
//
// Executes a single pre-planned block (produced by planChain()).  Entry and
// exit velocities are guaranteed achievable by the planner — no reactive
// junction clamping needed here.  The planner runs a simple trapezoidal
// (S-curve smoothed) profile:
//
//   Phase 1  – accelerate from entryVel toward cruiseVel
//   Phase 2  – coast at cruiseVel (may be absent for short segments)
//   Phase 3  – decelerate from cruiseVel to exitVel
//
// isComplete() fires on a simple position crossing — no velocity window.
// Any small velocity residual at the crossing is carried into the next block
// and corrected by the PD control loop.
// ---------------------------------------------------------------------------
class TrajectoryPlanner {
public:
    float currentPos = 0;
    float currentVel = 0;
    float currentAcc = 0;
    float targetPos  = 0;

private:
    float cruiseVel  = 0;
    float exitVel    = 0;
    float maxA       = 0;
    float jerk       = 0;
    bool  moveForward = true;

public:
    // Reset for a new pre-planned block.
    // currentPos/currentVel/currentAcc carry over from the previous block
    // (velocity continuity). The caller must zero currentVel before the
    // very first block.
    void resetForBlock(float startPos, float endPos, const PlannedBlock& blk) {
        currentPos  = startPos;
        targetPos   = endPos;
        cruiseVel   = blk.cruiseVel;
        exitVel     = blk.exitVel;
        maxA        = (blk.accel > 1.0f) ? blk.accel : 1.0f;
        jerk        = maxA * 100.0f;
        moveForward = blk.forward;
        currentAcc  = 0;
        // currentVel intentionally preserved for velocity continuity
    }

    void update(float dt) {
        float distToTarget = fabsf(targetPos - currentPos);
        float spd          = fabsf(currentVel);

        // Braking distance needed to decelerate from current speed to exitVel.
        float brakeDist = 0;
        if (spd > exitVel) {
            brakeDist = (spd * spd - exitVel * exitVel) / (2.0f * maxA);
        }

        float targetAcc = 0.0f;

        if (distToTarget < 5.0f && fabsf(spd - exitVel) < 20.0f) {
            // Damping zone: servo velocity to exitVel; if stopped short, nudge toward target.
            float signedExit = moveForward ? exitVel : -exitVel;
            targetAcc = (signedExit - currentVel) * 10.0f;
            // If motor has stopped short of target add a gentle position-proportional nudge
            // so isComplete() can fire rather than hanging at ~0 velocity.
            if (spd < 10.0f && distToTarget > 0.1f) {
                float nudge = moveForward ? (distToTarget * 300.0f) : -(distToTarget * 300.0f);
                targetAcc += nudge;
            }
            if (fabsf(targetAcc) > maxA) targetAcc = (targetAcc > 0) ? maxA : -maxA;
        } else if (brakeDist >= distToTarget - _brakeLookahead(spd)) {
            // Start braking with S-curve-aware lookahead (see _brakeLookahead).
            targetAcc = (currentVel > 0) ? -maxA : maxA;
        } else if (spd < cruiseVel) {
            // Accelerate to cruise speed
            targetAcc = moveForward ? maxA : -maxA;
        }
        // else: coast at cruiseVel

        // S-curve jerk limit
        if (currentAcc < targetAcc) {
            currentAcc += jerk * dt;
            if (currentAcc > targetAcc) currentAcc = targetAcc;
        } else if (currentAcc > targetAcc) {
            currentAcc -= jerk * dt;
            if (currentAcc < targetAcc) currentAcc = targetAcc;
        }

        currentVel += currentAcc * dt;
        // Clamp to ±cruiseVel
        if (currentVel >  cruiseVel) { currentVel =  cruiseVel; currentAcc = 0; }
        if (currentVel < -cruiseVel) { currentVel = -cruiseVel; currentAcc = 0; }

        currentPos += currentVel * dt;
    }

    // Position crossing with a small tolerance for zero-exit-vel blocks.
    // Allows isComplete() to fire when the motor stops fractionally short of the
    // target (≤3 steps) due to S-curve undershoot; the hold PD then corrects it.
    // For chained blocks with non-zero exitVel the tolerance is 0 (exact crossing).
    bool isComplete() const {
        float tol = (exitVel < 5.0f) ? 3.0f : 0.0f;
        return moveForward ? (currentPos >= targetPos - tol)
                           : (currentPos <= targetPos + tol);
    }

private:
    // Dynamic braking lookahead that accounts for the S-curve jerk ramp.
    //
    // When braking triggers, currentAcc must ramp from its current value down to
    // -maxA.  This ramp takes T = (currentAcc + maxA) / jerk seconds.  During
    // that window the motor continues at roughly constant velocity, traveling an
    // extra ~spd*T steps beyond what a hard-decel model predicts.
    //
    // Derivation (integrating the linear acc ramp):
    //   extra overshoot ≈ jerk * T² * (spd/(2*maxA) + T/3)
    //
    // Adding 2 steps of margin gives the motor a ≤2-step undershoot that the
    // damping zone + isComplete tolerance catch cleanly.
    float _brakeLookahead(float spd) const {
        float T = (currentAcc > -maxA) ? (currentAcc + maxA) / jerk : 0.0f;
        return jerk * T * T * (spd / (2.0f * maxA) + T / 3.0f) + 2.0f;
    }
};

// ---------------------------------------------------------------------------
// Marlin-style chain planner
//
// Computes globally-optimal entry/exit velocities for a sequence of moves
// using a forward pass (kinematic achievability) followed by a reverse pass
// (safe-stop propagation).
//
//   Forward pass:  entry[i+1] = min(desired_junction, sqrt(entry[i]^2 + 2*a[i]*d[i]))
//   Reverse pass:  entry[i+1] = min(entry[i+1], sqrt(exit[i+1]^2 + 2*a[i+1]*d[i+1]))
//                  exit[i]    = entry[i+1]
//   Feasibility:   if exit[i] is below the minimum achievable (motor can't decelerate
//                  fast enough), raise it to the kinematic minimum.
//
// cmds:     command array
// n:        command count
// startPos: absolute encoder position at chain start
// out:      output array (at least n elements)
// ---------------------------------------------------------------------------
static const int MAX_CHAIN_LEN = 32;

static void planChain(const MotionCommand* cmds, int n,
                      float startPos, PlannedBlock* out) {
    // ---- Populate blocks ----
    float pos = startPos;
    for (int i = 0; i < n; i++) {
        float raw = cmds[i].absolute
                    ? ((float)cmds[i].distance - pos)
                    : (float)cmds[i].distance;
        out[i].dist     = fabsf(raw);
        out[i].forward  = (raw >= 0.0f);
        out[i].cruiseVel = fabsf(cmds[i].maxSpeed);
        out[i].accel     = fabsf(cmds[i].acceleration);
        if (out[i].accel < 1.0f) out[i].accel = 1.0f;
        out[i].entryVel  = 0.0f;
        out[i].exitVel   = 0.0f;
        pos += raw;
    }

    // ---- Forward pass ----
    // Propagate maximum achievable entry velocity at each junction.
    out[0].entryVel = 0.0f; // chain always starts from rest
    for (int i = 1; i < n; i++) {
        bool sameDir = (out[i-1].forward == out[i].forward);
        float desired = sameDir
            ? fminf(out[i-1].cruiseVel, out[i].cruiseVel)
            : 0.0f; // direction reversal: must stop at boundary
        float maxReach = sqrtf(out[i-1].entryVel * out[i-1].entryVel
                               + 2.0f * out[i-1].accel * out[i-1].dist);
        out[i].entryVel = fminf(desired, maxReach);
        if (out[i].entryVel > out[i].cruiseVel) out[i].entryVel = out[i].cruiseVel;
    }

    // Provisional exit speeds = next block's entry (chain always ends at rest)
    for (int i = 0; i < n - 1; i++) out[i].exitVel = out[i+1].entryVel;
    out[n-1].exitVel = 0.0f;

    // ---- Reverse pass ----
    // Constrain entry speeds so the motor can always stop by chain end.
    for (int i = n - 2; i >= 0; i--) {
        float maxEntry = sqrtf(out[i+1].exitVel  * out[i+1].exitVel
                               + 2.0f * out[i+1].accel * out[i+1].dist);
        if (out[i+1].entryVel > maxEntry) out[i+1].entryVel = maxEntry;
        out[i].exitVel = out[i+1].entryVel; // propagate back
    }

    // ---- Feasibility clamp ----
    // After the reverse pass, a decelerating block's exit may have been lowered
    // below what maximum deceleration can achieve (rare: only when a very short
    // segment sits between two fast moves).  Raise exit to the physical minimum.
    for (int i = 0; i < n; i++) {
        if (out[i].entryVel > out[i].exitVel && out[i].dist > 0) {
            float sq = out[i].entryVel * out[i].entryVel
                       - 2.0f * out[i].accel * out[i].dist;
            float minExit = (sq > 0.0f) ? sqrtf(sq) : 0.0f;
            if (out[i].exitVel < minExit) {
                out[i].exitVel = minExit;
                if (i + 1 < n) out[i+1].entryVel = minExit;
            }
        }
        // exitVel cannot exceed cruiseVel
        if (out[i].exitVel > out[i].cruiseVel) out[i].exitVel = out[i].cruiseVel;

        Serial1.printf("DBG:PLAN[%d] dist=%.0f fwd=%d entry=%.0f cruise=%.0f exit=%.0f\n",
                       i, out[i].dist, (int)out[i].forward,
                       out[i].entryVel, out[i].cruiseVel, out[i].exitVel);
    }
}

// ---------------------------------------------------------------------------
// Diagnostics Task — Core 0, priority 2
//
// Runs at 10 Hz during motion, 1 Hz at rest (Option 4).
// Reads TMC UART registers (safe on Core 0 via TmcLock), assembles 0xAA 0xDD
// STATUS packets, and writes them on every cycle (Option 2 — no !running gate;
// UsbWriteGuard serialises all USBSerial writes, so concurrent UPDATE/STATUS
// writes interleave safely at the packet level).
// ---------------------------------------------------------------------------
static void DiagnosticsTask(void *) {
    for (;;) {
        // Option 4: 10 Hz during motion, 1 Hz at rest.
        vTaskDelay(pdMS_TO_TICKS(s_running ? 100 : 1000));

        // TMC UART reads — Core 0, TmcLock-protected inside each call
        tmc::DriverStatus ds = tmc::getDriverStatus();
        uint8_t  pwmScale    = (uint8_t)tmc::getPwmScaleSum();
        uint32_t tstep       = tmc::getInterstepDuration();
        uint16_t sgResult    = (uint16_t)tmc::getStallGuardResult();

        // Option 3: update inter-task cache for ControlTask telemetry assembly.
        // uint8_t stores are single-instruction atomic on ESP32 LX7.
        g_cs_actual_cache = (uint8_t)ds.current_scaling;
        g_pwm_scale_cache = pwmScale;

        // Cross-core reads — written by loop() on Core 1 at 1 Hz (volatile)
        float vbus = VBusVoltage;
        bool  pg   = (PGState == 0); // active-low on CH224K

        // System health
        uint16_t freeHeapKB = (uint16_t)(esp_get_free_heap_size() / 1024);
        uint16_t ctrlHWM    = s_controlHandle
                              ? (uint16_t)uxTaskGetStackHighWaterMark(s_controlHandle) : 0;
        uint16_t bootCnt    = bootCount > 65535u ? 0xFFFF : (uint16_t)bootCount;
        uint8_t  resetRsn   = (uint8_t)esp_reset_reason();

        // Motion / fault state (all static volatile — safe single-read)
        bool running     = s_running;
        bool holdActive  = g_hold_active;
        bool holdSettled = (g_hold_state == HOLD_SETTLED);
        bool brownout    = g_fault_brownout;
        bool lagFault    = g_fault_lag;

        // Serial1 diagnostic summary (AUX UART, always emitted)
        Serial1.printf("[DIAG] CS:%u/31 SC:%u OT:%u%s PWM:%u TSTEP:%lu SG:%u Heap:%ukB\r\n",
            ds.current_scaling, ds.stealth_chop ? 1 : 0,
            ds.over_temperature_warning ? 1 : 0,
            ds.over_temperature_shutdown ? " SHUTDOWN" : "",
            pwmScale, (unsigned long)tstep, sgResult, freeHeapKB);

        // Option 2: Send STATUS on every cycle regardless of motion state.
        // UsbWriteGuard ensures no byte-level interleaving with TelemetryTask.
        {
            uint8_t buf[20];
            buf[0] = 0xAA; buf[1] = 0xDD;

            uint16_t vbusMv = (uint16_t)(vbus * 1000.0f);
            buf[2] = vbusMv & 0xFF; buf[3] = vbusMv >> 8;

            uint8_t fa = 0;
            if (pg)                                fa |= (1 << 0); // PG OK
            if (ds.over_temperature_warning)       fa |= (1 << 1);
            if (ds.over_temperature_shutdown)      fa |= (1 << 2);
            if (lagFault)                          fa |= (1 << 3);
            if (brownout)                          fa |= (1 << 5);
            if (holdActive)                        fa |= (1 << 6);
            if (running)                           fa |= (1 << 7); // isRunning bit (Option 2)
            buf[4] = fa;

            uint8_t fb = 0;
            if (ds.stealth_chop)                   fb |= (1 << 0);
            if (ds.standstill)                     fb |= (1 << 1);
            if (ds.short_to_ground_a)              fb |= (1 << 3);
            if (ds.short_to_ground_b)              fb |= (1 << 4);
            if (ds.open_load_a)                    fb |= (1 << 5);
            if (ds.open_load_b)                    fb |= (1 << 6);
            if (holdSettled)                       fb |= (1 << 7);
            buf[5] = fb;

            buf[6] = (uint8_t)ds.current_scaling;
            buf[7] = pwmScale;

            uint16_t tstep16 = tstep > 65535u ? 0xFFFF : (uint16_t)tstep;
            buf[8] = tstep16 & 0xFF; buf[9] = tstep16 >> 8;

            buf[10] = sgResult & 0xFF; buf[11] = sgResult >> 8;
            buf[12] = freeHeapKB & 0xFF; buf[13] = freeHeapKB >> 8;
            buf[14] = ctrlHWM & 0xFF; buf[15] = ctrlHWM >> 8;
            buf[16] = bootCnt & 0xFF; buf[17] = bootCnt >> 8;
            buf[18] = resetRsn;

            // XOR checksum over bytes[2..18]
            uint8_t cs = 0;
            for (int i = 2; i < 19; i++) cs ^= buf[i];
            buf[19] = cs;

            UsbWriteGuard guard;
            if (guard) USBSerial.write(buf, 20);
        }
    }
}

// ---------------------------------------------------------------------------
// Telemetry Task — Core 0, priority 3, runs on demand
//
// Dequeues TelemetryData snapshots sent from ControlTask and forwards them
// to the TelemetryProvider (USBSerial.write). Running on Core 0 at low
// priority means a blocked USB CDC TX FIFO only stalls this task, not the
// 1 kHz ControlTask on Core 1.
// ---------------------------------------------------------------------------
static void TelemetryTask(void *) {
    TelemetryData d;
    for (;;) {
        if (xQueueReceive(s_teleQueue, &d, portMAX_DELAY) == pdPASS) {
            if (!s_telemetry) continue;
            if (d.type == TELEMETRY_STOP) {
                // All USBSerial writes happen here — PlannerTask enqueues rather
                // than calling sendStop() directly, preventing interleaved bytes.
                // Retry once after 500 ms if the first send fails (CDC TX may be
                // briefly disconnected due to a Windows USB CDC driver hiccup).
                Serial1.printf("DBG:STOP_SENDING pos=%ld\n", d.pos);
                size_t sent = s_telemetry->sendStop(d.stopReason, d.pos);
                if (sent == 0) {
                    Serial1.printf("WARN:STOP_SEND_FAILED — retrying in 500 ms\n");
                    vTaskDelay(pdMS_TO_TICKS(500));
                    sent = s_telemetry->sendStop(d.stopReason, d.pos);
                    if (sent == 0) {
                        Serial1.printf("ERR:STOP_NOT_SENT after retry\n");
                    }
                }
                Serial1.printf("DBG:STOP_SENT bytes=%u\n", (unsigned)sent);
            } else {
                s_telemetry->sendTelemetry(d);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Planner Task — Core 0, priority 5, runs at 500 Hz during a move
//
// Responsibilities:
//   • Receive MotionCommand from queue
//   • Generate trajectory via TrajectoryPlanner and push to trajbuf
//   • Check VBus (brownout) and StallGuard (200 ms intervals) — uses UART/ADC,
//     safe on Core 0, must NOT be called from the Control Task
//   • Detect move completion; handle all TMC enable/disable calls
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Planner Task — Core 0, priority 5, runs at 500 Hz during a move
//
// Offline block planner (Marlin-style):
//   1. Wait for first command → enable TMC → 20 ms settle window
//   2. Drain ALL queued commands into cmdBuf (up to MAX_CHAIN_LEN)
//   3. planChain() → globally optimal block velocities (forward+reverse pass)
//   4. Execute blocks sequentially; carry velocity/position across boundaries
//   5. Single STOP packet after all blocks or on fault
//
// VBus brownout and StallGuard are checked at 200 ms intervals (UART/ADC
// safe on Core 0; must NOT be called from ControlTask on Core 1).
// ---------------------------------------------------------------------------
static void PlannerTask(void *) {
    MotionCommand    cmdBuf[MAX_CHAIN_LEN];
    PlannedBlock     blocks[MAX_CHAIN_LEN];
    TrajectoryPlanner planner;

    float uSteps          = 32.0f;
    float counts_to_steps = (200.0f * uSteps) / 4096.0f;
    (void)counts_to_steps; // updated per-chain; referenced via g_uSteps_setting in ControlTask

    for (;;) {
        // ---- Wait for first command ----
        if (xQueueReceive(s_motionQueue, &cmdBuf[0], portMAX_DELAY) != pdPASS) continue;

        // ---- TMC enable (first move only) + settle ----
        uSteps          = (float)g_uSteps_setting;
        if (uSteps < 1) uSteps = 32.0f;
        counts_to_steps = (200.0f * uSteps) / 4096.0f;

        g_hold_active = false; // suspend active hold during move
        g_hold_state = HOLD_CORRECTING; // reset state machine for next hold
        g_settle_start_ms = 0;
        if (!s_driver_enabled) {
            tmc::enable();
            vTaskDelay(pdMS_TO_TICKS(20)); // wait for driver rails to stabilise
            s_driver_enabled = true;
        }

        // ---- Drain all queued commands ----
        // If the last received command has chain=true, wait up to 50 ms for
        // the next command to arrive (the serial parser may not have enqueued
        // it yet).  Without this, back-to-back chain commands sent from the
        // host can be split into separate single-command executions.
        int nCmds = 1;
        while (nCmds < MAX_CHAIN_LEN) {
            TickType_t wait = cmdBuf[nCmds - 1].chain ? pdMS_TO_TICKS(50) : 0;
            if (xQueueReceive(s_motionQueue, &cmdBuf[nCmds], wait) == pdPASS) {
                nCmds++;
            } else {
                break;
            }
        }
        Serial1.printf("DBG:PLANNER %d cmd(s) queued\n", nCmds);

        // ---- planChain: compute globally-optimal block velocities ----
        float chainStartPos = g_meas_pos;
        planChain(cmdBuf, nCmds, chainStartPos, blocks);

        // ---- Reset faults and drain any stale telemetry from the previous run ----
        // PlannerTask (pri 5) never yields between enqueueing the previous STOP and
        // picking up a new command (both are on Core 0 with no blocking call between).
        // xQueueReset here ensures TelemetryTask cannot deliver a previous run's STOP
        // packet while this run is already producing UPDATE packets.  Safe to call:
        // ControlTask does not enqueue UPDATE packets until s_running is true (set below).
        g_fault_lag = g_fault_estop = g_fault_estop_gui = g_fault_brownout = false;
        xQueueReset(s_teleQueue);
        s_running   = true;

        char     stopReason[32] = "Completed";
        uint32_t lastVBusMs    = millis();
        uint32_t lastSGMs      = millis();
        uint32_t prevPlanUs    = micros();
        TickType_t xLastWake   = xTaskGetTickCount();

        // ---- Execute each block sequentially ----
        float blockStartPos = chainStartPos;
        planner.currentVel = 0.0f;
        planner.currentAcc = 0.0f;

        for (int bi = 0; bi < nCmds && s_running; bi++) {
            const PlannedBlock& blk = blocks[bi];

            float blockEndPos = blockStartPos + (blk.forward ? blk.dist : -blk.dist);
            planner.resetForBlock(blockStartPos, blockEndPos, blk);
            g_target_pos = blockEndPos;

            trajbuf::clear();
            prevPlanUs = micros();
            xLastWake  = xTaskGetTickCount();

            Serial1.printf("DBG:BLOCK[%d] start=%.0f end=%.0f entry=%.0f exit=%.0f\n",
                           bi, blockStartPos, blockEndPos, blk.entryVel, blk.exitVel);

            // -- Inner loop: run this block at 500 Hz --
            while (s_running) {
                uint32_t nowUs = micros();
                float dt = (float)(nowUs - prevPlanUs) * 1e-6f;
                if (dt > 0.005f) dt = 0.005f;
                if (dt < 0.0001f) dt = 0.001f;
                prevPlanUs = nowUs;

                planner.update(dt);

                TrajectoryPoint pt;
                pt.pos = planner.currentPos;
                pt.vel = planner.currentVel;
                pt.acc = planner.currentAcc;
                trajbuf::push(pt);

                // VBus brownout check (200 ms)
                if (millis() - lastVBusMs >= 200) {
                    lastVBusMs = millis();
                    float vbus_mv = (float)analogReadMilliVolts(VBUS_PIN);
                    float vbus    = (vbus_mv / 1000.0f) / DIV_RATIO;
                    if (vbus < g_brownout_threshold_v) g_fault_brownout = true;
                }

                // StallGuard check (200 ms)
                if (millis() - lastSGMs >= 200) {
                    lastSGMs    = millis();
                    g_sg_result = (uint16_t)tmc::getStallGuardResult();
                }

                // Fault handling
                if (g_fault_lag)          { strncpy(stopReason, "Lag Fault",      31); s_running = false; }
                if (g_fault_estop)        { strncpy(stopReason, "E-STOP (SW1)",   31); s_running = false; }
                if (g_fault_estop_gui)    { strncpy(stopReason, "E-STOP (GUI)",   31); s_running = false; }
                if (g_fault_brownout)     { strncpy(stopReason, "Brownout Fault", 31); s_running = false; }
                if (!s_running) break;

                // Block complete: position crossing (with small zero-exit-vel tolerance)
                if (planner.isComplete()) {
                    Serial1.printf("DBG:BLOCK_DONE vel=%.1f pos=%.1f tgt=%.1f\n",
                                   planner.currentVel, planner.currentPos, planner.targetPos);
                    break;
                }

                vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
            }

            // Carry planner position to next block start (velocity is already live in planner)
            blockStartPos = planner.currentPos;

            // Option C streaming hook: if more commands arrive here, append to blocks[] and
            // re-run planChain() over the remaining+new commands for seamless continuation.
        }

        // ---- Exit: stop trajectory following immediately ----
        s_running = false;  // ControlTask stops issuing trajectory commands on next tick
        stepgen::halt();
        Serial1.printf("DBG:PLANNER_DONE reason=%s\n", stopReason);

        bool faulted = g_fault_lag || g_fault_estop || g_fault_estop_gui || g_fault_brownout;
        if (!faulted) {
            // Normal completion — enter active hold so PD loop corrects drift
            g_hold_target = g_target_pos;
            g_hold_active = true;
        } else {
            // Fault — disable driver for safety, do NOT enter active hold
            g_hold_active = false;
            tmc::disable();
            s_driver_enabled = false;
            Serial1.printf("DBG:FAULT_TMC_DISABLED\n");
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // brief settle

        // Route STOP through the telemetry queue so TelemetryTask owns all
        // USBSerial writes.  Calling sendStop() directly here (from PlannerTask)
        // while TelemetryTask may concurrently be inside sendTelemetry() causes
        // interleaved bytes on the USB CDC TX buffer — Python never sees a clean
        // 0xAA 0xCC header.  Using the queue serialises the writes by FIFO order.
        if (s_teleQueue) {
            TelemetryData stopData = {};
            stopData.type = TELEMETRY_STOP;
            stopData.pos  = (long)g_meas_pos;
            strncpy(stopData.stopReason, stopReason, sizeof(stopData.stopReason) - 1);
            Serial1.printf("DBG:STOP_QUEUED pos=%ld reason=%s\n", (long)g_meas_pos, stopReason);
            if (xQueueSend(s_teleQueue, &stopData, pdMS_TO_TICKS(200)) != pdPASS) {
                Serial1.printf("ERR:STOP_QUEUE_FULL — STOP packet dropped!\n");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Control Task — Core 1, priority 19, runs at 1 kHz continuously
//
// Responsibilities:
//   • Read encoder position (via volatile counter — no I2C/UART)
//   • Pop latest trajectory reference from trajbuf
//   • Compute PD + feedforward velocity command
//   • Drive step generator
//   • Detect lag fault and E-stop (sets shared flags, halts steps immediately)
//   • Emit binary telemetry at 10 Hz via TelemetryProvider
//
// IMPORTANT: This task MUST NOT call tmc:: UART functions, encoder::read(),
// or any blocking API. Only encoder::getTotalCounts() (reads a volatile) is
// permitted.
// ---------------------------------------------------------------------------
static void ControlTask(void *) {
    // ControlTask is not registered with the task WDT by default.
    // The delete call was a no-op and is removed.

    PDController pd;
    TrajectoryPoint ref = {0.0f, 0.0f, 0.0f};

    float uSteps          = 32.0f;
    float counts_to_steps = (200.0f * uSteps) / 4096.0f;

    uint32_t lastTeleMs   = millis();
    long     lastTeleEnc  = 0;
    uint8_t  estopCount   = 0; // consecutive LOW reads needed to trip E-stop

    // Hold-phase velocity: accumulate encoder delta over 100 ms windows so that
    // the velocity estimate has 10× better resolution than the 10 ms packet rate.
    // At 32 µsteps: 1 count / 10 ms = 156 steps/s; 1 count / 100 ms = 15.6 steps/s.
    long     holdVelRefEnc = 0;
    uint32_t holdVelRefMs  = 0;
    float    holdVelEst    = 0.0f;

    // AUX diagnostic state for hold-phase telemetry rate verification
    uint32_t holdDiagCount  = 0;
    uint32_t holdDiagLastMs = 0;

    TickType_t xLastWake  = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(1)); // 1 kHz

        // --- Encoder read (pure volatile — no I2C here) ---
        uSteps          = (float)g_uSteps_setting;
        if (uSteps < 1) uSteps = 32.0f;
        counts_to_steps = (200.0f * uSteps) / 4096.0f;

        long  encCounts = encoder::getTotalCounts();
        float measPos   = (float)encCounts * counts_to_steps;
        g_meas_pos      = measPos; // share with Planner Task

        if (!s_running) {
            if (g_hold_active) {
                // Hold state machine: CORRECTING → SETTLED
                //
                // CORRECTING: PD loop drives corrections when position is
                //   outside the deadband. Step pulses keep TMC2209 at IRUN.
                // SETTLED: position stayed inside deadband for HOLD_SETTLE_MS.
                //   Step pulses stop → TMC2209 detects standstill → drops to
                //   IHOLD automatically, dramatically reducing motor heat.
                const float holdDeadband = 4.0f * counts_to_steps;
                float holdError = g_hold_target - measPos;
                bool insideDeadband = (fabsf(holdError) <= holdDeadband);

                if (g_hold_state == HOLD_SETTLED) {
                    // In SETTLED state — no step pulses, TMC2209 handles holding
                    if (!insideDeadband) {
                        // Position drifted out — re-enter correcting
                        g_hold_state = HOLD_CORRECTING;
                        g_settle_start_ms = 0;
                    } else {
                        stepgen::setVelocity(0.0f);
                    }
                }

                if (g_hold_state == HOLD_CORRECTING) {
                    if (insideDeadband) {
                        // Start or continue the settle timer
                        if (g_settle_start_ms == 0) {
                            g_settle_start_ms = millis();
                        } else if ((millis() - g_settle_start_ms) >= HOLD_SETTLE_MS) {
                            // Transition to SETTLED
                            g_hold_state = HOLD_SETTLED;
                            stepgen::setVelocity(0.0f);
                            pd.prev_error = 0.0f;
                        }
                        // While counting down, stay silent (already in deadband)
                        stepgen::setVelocity(0.0f);
                        pd.prev_error = 0.0f;
                    } else {
                        // Outside deadband — correct and reset settle timer
                        g_settle_start_ms = 0;
                        float correction = pd.compute(g_hold_target, 0.0f, measPos, 0.001f);
                        stepgen::setVelocity(correction);
                    }
                }
                // Hold telemetry: 100 Hz while correcting (matches motion rate so the
                // settle transient is fully visible in the chart), 10 Hz once settled.
                const uint32_t holdTeleInterval = (g_hold_state == HOLD_CORRECTING) ? 10 : 100;

                // Velocity window: accumulate encoder delta over 100 ms regardless of
                // packet rate. This gives 10× lower quantization noise vs 10 ms window.
                // (1 encoder count / 100 ms = 15.6 steps/s at 32 µsteps, vs 156 at 10 ms)
                if (holdVelRefMs == 0) {
                    // First entry into hold: seed the window from current position
                    holdVelRefEnc = encCounts;
                    holdVelRefMs  = millis();
                }
                if (millis() - holdVelRefMs >= 100) {
                    float velDt = (millis() - holdVelRefMs) / 1000.0f;
                    holdVelEst  = (float)(encCounts - holdVelRefEnc)
                                  * counts_to_steps / (velDt > 0.001f ? velDt : 0.1f);
                    holdVelRefEnc = encCounts;
                    holdVelRefMs  = millis();
                }

                if (millis() - lastTeleMs >= holdTeleInterval) {
                    lastTeleMs  = millis();
                    lastTeleEnc = encCounts; // keep in sync (used by motion path after next move)

                    if (s_teleQueue) {
                        TelemetryData d;
                        d.type      = TELEMETRY_UPDATE;
                        d.timestamp = micros();
                        d.pos       = stepgen::getStepCount();
                        d.meas      = (long)measPos;
                        d.target    = (long)g_hold_target;
                        d.lag       = (int)(g_hold_target - measPos);
                        d.vel       = (int)holdVelEst;  // 100 ms window → low quantization noise
                        d.p_acc     = 0;
                        d.p_dist    = 0;
                        d.sg_result = g_sg_result;
                        d.cs_actual = g_cs_actual_cache;
                        d.pwm_scale = g_pwm_scale_cache;
                        d.mvel      = (int16_t)holdVelEst;
                        xQueueSend(s_teleQueue, &d, 0);
                    }

                    // AUX diagnostic: log hold tele rate and smoothed velocity.
                    // Prints on packet #1 (hold entry), then every 20 packets.
                    holdDiagCount++;
                    if (holdDiagCount == 1 || holdDiagCount % 20 == 0) {
                        uint32_t gapMs = (holdDiagLastMs > 0)
                                         ? (uint32_t)(millis() - holdDiagLastMs) * 20
                                         : 0;
                        Serial1.printf("DBG:HOLD #%lu interval=%lu vel=%.1f lag=%d state=%s gap=%lums\n",
                                       holdDiagCount, holdTeleInterval, holdVelEst,
                                       (int)(g_hold_target - measPos),
                                       g_hold_state == HOLD_CORRECTING ? "CORR" : "SETT",
                                       gapMs);
                        holdDiagLastMs = millis();
                    }
                }
            } else {
                stepgen::setVelocity(0.0f);
                pd.reset();
                g_hold_state     = HOLD_CORRECTING;
                g_settle_start_ms = 0; // ensure clean state for next hold activation
                holdVelRefMs     = 0;  // reset velocity window so first entry re-seeds
                holdVelEst       = 0.0f;
                holdDiagCount    = 0;  // reset per-hold diagnostic counter
                holdDiagLastMs   = 0;
            }
            continue;
        }

        // --- Refresh PD gains (written infrequently by main task at rest) ---
        pd.setGains(g_kp, g_kd);
        pd.setPhaseLeadGain(g_kv);

        // --- Consume latest trajectory point (reuse last if buffer empty) ---
        TrajectoryPoint newRef;
        if (trajbuf::pop(newRef)) {
            ref = newRef;
        }

        // --- PD + feedforward velocity command ---
        float correction  = pd.compute(ref.pos, ref.vel, measPos, 0.001f);
        float velocity_cmd = ref.vel + correction;
        stepgen::setVelocity(velocity_cmd);

        // --- E-Stop (SW1 button — active LOW, debounced over 5 consecutive 1ms ticks) ---
        // A single glitch at 1 kHz sampling would otherwise trip an irreversible fault.
        if (digitalRead(SW1_PIN) == LOW) {
            if (++estopCount >= 5) {
                g_fault_estop = true;
                stepgen::halt();
            }
        } else {
            estopCount = 0;
        }

        // --- Lag fault: encoder position has drifted too far from reference ---
        float phase_error = fabsf(ref.pos - measPos);
        if (phase_error > 200.0f * uSteps * 1.5f) {
            g_fault_lag = true;
            stepgen::halt();
        }

        // --- Telemetry at 100 Hz — enqueue snapshot for TelemetryTask ---
        if (millis() - lastTeleMs >= 10) {
            float dt_s    = (millis() - lastTeleMs) / 1000.0f;
            lastTeleMs    = millis();
            float mVel    = (float)(encCounts - lastTeleEnc)
                            * counts_to_steps / dt_s;
            lastTeleEnc   = encCounts;

            if (s_teleQueue) {
                TelemetryData d;
                d.type      = TELEMETRY_UPDATE;
                d.timestamp = micros();
                d.pos       = stepgen::getStepCount();
                d.meas      = (long)measPos;
                d.target    = (long)ref.pos;
                d.lag       = (int)(ref.pos - measPos);
                d.vel       = (int)ref.vel;
                d.p_acc     = (int)ref.acc;
                d.p_dist    = (int)(g_target_pos - measPos);
                d.sg_result = g_sg_result;
                d.cs_actual = g_cs_actual_cache;  // Option 3: TMC cache (atomic uint8 read)
                d.pwm_scale = g_pwm_scale_cache;  // Option 3: TMC cache (atomic uint8 read)
                d.mvel      = (int16_t)mVel;
                // Non-blocking: drop packet if queue full rather than stalling.
                xQueueSend(s_teleQueue, &d, 0);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void setTelemetryProvider(TelemetryProvider *provider) {
    s_telemetry = provider;
    if (provider) provider->init();
}

void setPD(float kp, float kd) {
    g_kp = kp;
    g_kd = kd;
}

void setPhaseLeadGain(float kv) {
    g_kv = kv;
}

void setConfiguredVoltage(float volts) {
    // Brownout threshold = 70% of configured supply.
    // USB-PD voltages are within ±5% in steady state; 70% gives enough margin
    // to catch a genuine dropout without false-tripping under motor load.
    if (volts > 4.0f) {
        g_brownout_threshold_v = volts * 0.7f;
    }
}

void setMicrosteps(int ms) {
    if (ms < 1) ms = 1;
    g_uSteps_setting = (int32_t)ms;
}

void init() {
    s_motionQueue = xQueueCreate(10, sizeof(MotionCommand));
    s_teleQueue   = xQueueCreate(4,  sizeof(TelemetryData)); // 4 slots: headroom at 100 Hz

    // Initialise step generator ISR (timer starts immediately but produces no
    // pulses until setVelocity() is called with a non-zero value).
    stepgen::init(STEP_PIN, DIR_PIN);

    // NOTE: run current is applied by configureSettings() in main.cpp setup()
    // after motion::init(). Do NOT set it here — it would override user-saved settings.

    // Telemetry Task: Core 0, lowest priority — allowed to block on USB TX
    xTaskCreatePinnedToCore(TelemetryTask,    "TeleTask",   2048, nullptr,  3,
                            &s_teleHandle,    0);

    // Diagnostics Task: Core 0, low priority — 1 Hz TMC UART + STATUS packet
    xTaskCreatePinnedToCore(DiagnosticsTask,  "DiagTask",   4096, nullptr,  2,
                            &s_diagHandle,    0);

    // Planner Task: Core 0, lower priority — can use UART/ADC safely
    xTaskCreatePinnedToCore(PlannerTask,   "PlannerTask", 8192, nullptr,  5,
                            &s_plannerHandle, 0);

    // Control Task: Core 1, high priority — timing-critical real-time loop
    xTaskCreatePinnedToCore(ControlTask,   "ControlTask", 4096, nullptr, 19,
                            &s_controlHandle, 1);
}

bool addCommand(long distance, float acceleration, float maxSpeed, bool absolute, bool chain) {
    // Soft limits — prevent commands from exceeding hardware capabilities.
    // Max step rate: stepgen ISR at 40 kHz (one pulse per tick).
    // Max acceleration: practical limit to avoid immediate lag faults at rest.
    constexpr float MAX_SPEED_STEPS  = 38000.0f; // slightly below ISR rate
    constexpr float MAX_ACCEL_STEPS  = 200000.0f;
    constexpr float MIN_SPEED_STEPS  = 10.0f;
    constexpr float MIN_ACCEL_STEPS  = 10.0f;

    if (maxSpeed    > MAX_SPEED_STEPS) maxSpeed    = MAX_SPEED_STEPS;
    if (maxSpeed    < MIN_SPEED_STEPS) maxSpeed    = MIN_SPEED_STEPS;
    if (acceleration > MAX_ACCEL_STEPS) acceleration = MAX_ACCEL_STEPS;
    if (acceleration < MIN_ACCEL_STEPS) acceleration = MIN_ACCEL_STEPS;

    MotionCommand cmd = {distance, acceleration, maxSpeed, absolute, chain};
    return xQueueSend(s_motionQueue, &cmd, 0) == pdPASS;
}

bool isRunning() { return s_running; }
bool isHoldActive() { return g_hold_active; }
float getHoldTarget() { return g_hold_target; }
uint8_t getHoldState() { return g_hold_state; }

bool isBrownoutFault() { return g_fault_brownout; }
bool isLagFault()      { return g_fault_lag; }
bool isEstopFault()    { return g_fault_estop || g_fault_estop_gui; }

void triggerEstop()    { g_fault_estop_gui = true; }

} // namespace motion
