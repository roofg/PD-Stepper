#include "motion_control.h"
#include "encoder.h"
#include "pd_controller.h"
#include "pins.h"
#include "step_generator.h"
#include "telemetry_provider.h"
#include "tmc_driver.h"
#include "trajectory_buffer.h"
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define STEP_PIN   TMC_STEP
#define DIR_PIN    TMC_DIR
#define SW1_PIN    PIN_SW1
#define VBUS_PIN   PIN_VBUS
#define DIV_RATIO  0.1189427313f

namespace motion {

// ---------------------------------------------------------------------------
// Inter-task shared state
// All are 32-bit aligned (or bool = 8-bit atomic on ESP32). No mutex needed
// for SPSC access patterns described in comments.
// ---------------------------------------------------------------------------

static volatile bool    s_running        = false; // set by planner, cleared by planner/control
static volatile float   g_meas_pos       = 0.0f;  // written: control; read: planner
static volatile float   g_target_pos     = 0.0f;  // written: planner; read: control (telemetry)
static volatile bool    g_fault_lag      = false;  // set: control; cleared: planner on move start
static volatile bool    g_fault_estop    = false;  // set: control; cleared: planner on move start
static volatile bool    g_fault_brownout = false;  // set: planner; cleared: planner on move start
static volatile uint16_t g_sg_result     = 0;      // written: planner (UART); read: control (tele)
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

static QueueHandle_t     s_motionQueue   = nullptr;
static QueueHandle_t     s_teleQueue     = nullptr; // ControlTask → TelemetryTask
static TaskHandle_t      s_plannerHandle = nullptr;
static TaskHandle_t      s_controlHandle = nullptr;
static TaskHandle_t      s_teleHandle    = nullptr;
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

        if (distToTarget < 2.0f && fabsf(spd - exitVel) < 20.0f) {
            // Damping zone: servo velocity smoothly to exitVel
            float signedExit = moveForward ? exitVel : -exitVel;
            targetAcc = (signedExit - currentVel) * 10.0f;
            if (fabsf(targetAcc) > maxA) targetAcc = (targetAcc > 0) ? maxA : -maxA;
        } else if (brakeDist >= distToTarget - spd * 0.002f) {
            // Start braking (one-tick lookahead buffer prevents overshoot)
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

    // Simple position crossing — no velocity gate.
    bool isComplete() const {
        return moveForward ? (currentPos >= targetPos) : (currentPos <= targetPos);
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
// Telemetry Task — Core 0, priority 3, runs on demand
//
// Dequeues TelemetryData snapshots sent from ControlTask and forwards them
// to the TelemetryProvider (USBSerial.write). Running on Core 0 at low
// priority means a blocked USB CDC TX FIFO only stalls this task, not the
// 1 kHz ControlTask on Core 1.
// ---------------------------------------------------------------------------
static void TelemetryTask(void *) {    TelemetryData d;
    for (;;) {
        if (xQueueReceive(s_teleQueue, &d, portMAX_DELAY) == pdPASS) {
            if (s_telemetry) s_telemetry->sendTelemetry(d);
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

        // ---- TMC enable + settle ----
        uSteps          = (float)g_uSteps_setting;
        if (uSteps < 1) uSteps = 32.0f;
        counts_to_steps = (200.0f * uSteps) / 4096.0f;

        tmc::enable();
        vTaskDelay(pdMS_TO_TICKS(20)); // wait for driver rails to stabilise

        // ---- Drain all queued commands within the settle window ----
        int nCmds = 1;
        while (nCmds < MAX_CHAIN_LEN) {
            if (xQueueReceive(s_motionQueue, &cmdBuf[nCmds], 0) == pdPASS) {
                nCmds++;
            } else {
                break; // queue empty
            }
        }
        Serial1.printf("DBG:PLANNER %d cmd(s) queued\n", nCmds);

        // ---- planChain: compute globally-optimal block velocities ----
        float chainStartPos = g_meas_pos;
        planChain(cmdBuf, nCmds, chainStartPos, blocks);

        // ---- Reset faults ----
        g_fault_lag = g_fault_estop = g_fault_brownout = false;
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
                if (g_fault_lag)      { strncpy(stopReason, "Lag Fault",      31); s_running = false; }
                if (g_fault_estop)    { strncpy(stopReason, "E-STOP (SW1)",   31); s_running = false; }
                if (g_fault_brownout) { strncpy(stopReason, "Brownout Fault", 31); s_running = false; }
                if (!s_running) break;

                // Block complete: simple position crossing — no velocity window needed
                if (planner.isComplete()) break;

                vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
            }

            // Carry planner position to next block start (velocity is already live in planner)
            blockStartPos = planner.currentPos;

            // Option C streaming hook: if more commands arrive here, append to blocks[] and
            // re-run planChain() over the remaining+new commands for seamless continuation.
        }

        // ---- Clean exit ----
        Serial1.printf("DBG:PLANNER_DONE reason=%s\n", stopReason);

        stepgen::halt();
        vTaskDelay(pdMS_TO_TICKS(100));

        Serial1.printf("DBG:TMC_DISABLE_START\n");
        tmc::disable();
        Serial1.printf("DBG:TMC_DISABLE_DONE\n");

        if (s_telemetry) {
            Serial1.printf("DBG:STOP_SENDING pos=%ld\n", (long)g_meas_pos);
            s_telemetry->sendStop(stopReason, (long)g_meas_pos);
            Serial1.printf("DBG:STOP_SENT\n");
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
            stepgen::setVelocity(0.0f);
            pd.reset();
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

        // --- Telemetry at 10 Hz — enqueue snapshot for TelemetryTask ---
        if (millis() - lastTeleMs >= 100) {
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
    s_teleQueue   = xQueueCreate(2,  sizeof(TelemetryData)); // 2 slots: 1 active + 1 slack

    // Initialise step generator ISR (timer starts immediately but produces no
    // pulses until setVelocity() is called with a non-zero value).
    stepgen::init(STEP_PIN, DIR_PIN);

    tmc::setRunCurrent(80); // 80 % run current for safe high-speed moves

    // Telemetry Task: Core 0, lowest priority — allowed to block on USB TX
    xTaskCreatePinnedToCore(TelemetryTask, "TeleTask",   2048, nullptr,  3,
                            &s_teleHandle,    0);

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

} // namespace motion
