#include "motion_control.h"
#include "encoder.h"
#include "homing.h"
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
static volatile float g_kp      = 3.0f;
static volatile float g_kd      = 0.1f;
static volatile float g_kv      = 0.0f;
static volatile float g_ka      = 0.0f;  // acceleration feedforward gain (seconds)
static volatile float g_d_alpha = 0.8f;  // D-term EMA filter coefficient
static volatile float g_jerk        = 0.0f;  // µsteps/s³; 0 = auto (maxA * 100)
static volatile float g_jerk_ramp_s = 0.0f;  // ramp time in seconds; 0 = use g_jerk

// Brownout threshold — 70% of the configured USB-PD supply voltage.
// Written once from setup() via setConfiguredVoltage(); read by PlannerTask.
static volatile float g_brownout_threshold_v = 9.0f; // safe default (12V * 0.75)

// Active hold — when true, ControlTask runs PD toward g_hold_target instead
// of zeroing velocity. Written by PlannerTask; read by ControlTask.
static volatile bool  g_hold_active    = false;
static volatile float g_hold_target    = 0.0f;
// Hold deadband in encoder counts. PD loop corrects only when |error| exceeds
// this many counts. Default 4 = ±33 µm at 34.18 mm/rev. Min safe value ~2.
static volatile float g_hold_deadband_counts = 4.0f;
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
// Loop pattern storage + planner state machine
// ---------------------------------------------------------------------------
static volatile PlannerState s_plannerState = PLANNER_IDLE;
static MotionCommand   s_loopPattern[planner::MAX_CHAIN_LEN];
static volatile int    s_loopLen       = 0;
static volatile float  s_loopWrapVel   = 0.0f; // steady-state wrap junction velocity
static volatile bool   s_stopRequested = false; // signal from stopLoop()/controlledStop()

// ---------------------------------------------------------------------------
// Trajectory Planner and Chain Planner — now in planner_core.h
// Local aliases for convenience.
// ---------------------------------------------------------------------------
using planner::TrajectoryPlanner;
using planner::JerkConfig;

// Build a JerkConfig from the current volatile globals.
static JerkConfig currentJerkConfig() {
    return { g_jerk, g_jerk_ramp_s };
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
        vTaskDelay(pdMS_TO_TICKS((s_running || homing::isActive()) ? 100 : 1000));

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
            if (homing::isActive())                 fa |= (1 << 4); // isHoming
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
            } else if (d.type == TELEMETRY_BLOCK_DONE) {
                s_telemetry->sendBlockDone(d.blockIndex, d.totalBlocks, d.pos);
            } else if (d.type == TELEMETRY_QUEUE_STATUS) {
                s_telemetry->sendQueueStatus(d.queueSlots, d.plannerState);
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
// Continuous event-loop planner (Marlin-style look-ahead):
//   1. Wait for first command → enable TMC → 20 ms settle window
//   2. Append to ring buffer, plan, begin executing immediately
//   3. On each 500 Hz tick: accept new commands, replan, generate trajectory
//   4. Advance head when block completes; seamless velocity carry-over
//   5. Single STOP packet when ring empties or on fault
//
// VBus brownout and StallGuard are checked at 200 ms intervals (UART/ADC
// safe on Core 0; must NOT be called from ControlTask on Core 1).
// ---------------------------------------------------------------------------

// Helper: convert a MotionCommand to a BlockEntry and append to ring.
// tailPos is updated to the end position of the new block.
static void appendCommandToRing(const MotionCommand& cmd,
                                planner::BlockRingBuffer& ring,
                                float& tailPos) {
    planner::BlockEntry entry = {};
    float raw = cmd.absolute
                ? ((float)cmd.distance - tailPos)
                : (float)cmd.distance;
    entry.plan.dist      = std::fabs(raw);
    entry.plan.forward   = (raw >= 0.0f);
    entry.plan.cruiseVel = std::fabs(cmd.maxSpeed);
    entry.plan.accel     = std::fabs(cmd.acceleration);
    if (entry.plan.accel < 1.0f) entry.plan.accel = 1.0f;
    entry.plan.entryVel  = 0.0f;
    entry.plan.exitVel   = 0.0f;
    entry.startPos = tailPos;
    tailPos += raw;
    entry.endPos = tailPos;
    ring.append(entry);
}

static void PlannerTask(void *) {
    MotionCommand cmd;
    planner::BlockRingBuffer ring;
    TrajectoryPlanner planner;

    float uSteps          = 32.0f;
    float counts_to_steps = (200.0f * uSteps) / 4096.0f;

    for (;;) {
        // ---- Wait for first command ----
        if (xQueueReceive(s_motionQueue, &cmd, portMAX_DELAY) != pdPASS) continue;

        // ---- TMC enable (first move only) + settle ----
        uSteps          = (float)g_uSteps_setting;
        if (uSteps < 1) uSteps = 32.0f;
        counts_to_steps = (200.0f * uSteps) / 4096.0f;

        g_hold_active = false; // suspend active hold during move
        g_hold_state = HOLD_CORRECTING;
        g_settle_start_ms = 0;
        if (!s_driver_enabled) {
            tmc::enable();
            vTaskDelay(pdMS_TO_TICKS(20));
            s_driver_enabled = true;
        }

        // ---- Determine mode: loop or streaming ----
        bool isLoop = (s_plannerState == PLANNER_LOOP_RUNNING);
        int  loopIdx = 0;     // next pattern index for refill
        float wrapVel = 0.0f; // steady-state loop wrap junction velocity

        float chainStartPos = g_hold_target;
        float tailPos = chainStartPos;
        ring.clear();

        if (isLoop) {
            // Loop mode: fill ring from stored pattern
            int loopLen = s_loopLen;
            while (!ring.isFull() && loopIdx < loopLen) {
                appendCommandToRing(s_loopPattern[loopIdx], ring, tailPos);
                loopIdx++;
            }
            // Keep filling with subsequent pattern iterations
            while (!ring.isFull()) {
                appendCommandToRing(s_loopPattern[loopIdx % loopLen], ring, tailPos);
                loopIdx++;
            }

            // Compute steady-state wrap junction velocity using planLoop()
            // on a single period of the pattern.
            {
                planner::BlockRingBuffer tempRing;
                float tempTail = 0.0f;
                for (int i = 0; i < loopLen; i++) {
                    appendCommandToRing(s_loopPattern[i], tempRing, tempTail);
                }
                planner::planLoop(tempRing);
                wrapVel = tempRing.at(0).plan.entryVel;
                s_loopWrapVel = wrapVel;
            }

            // Plan the full ring: starts from rest, tail connects to next iteration
            planner::incrementalPlan(ring, 0.0f, wrapVel);
            Serial1.printf("DBG:LOOP_START %d pattern cmds, wrapVel=%.0f, ring=%d\n",
                           loopLen, wrapVel, ring.count());
        } else {
            // Streaming mode: fill from command queue.
            // If the latest command has chain=true, wait up to 10 ms for the
            // next command — the serial parser may not have enqueued it yet.
            // Without this, back-to-back chain commands from the host can be
            // split into separate single-command executions.
            s_plannerState = PLANNER_RUNNING;
            appendCommandToRing(cmd, ring, tailPos);
            while (!ring.isFull()) {
                TickType_t wait = cmd.chain ? pdMS_TO_TICKS(10) : 0;
                if (xQueueReceive(s_motionQueue, &cmd, wait) == pdPASS) {
                    appendCommandToRing(cmd, ring, tailPos);
                } else {
                    break;
                }
            }
            planner::incrementalPlan(ring);
            Serial1.printf("DBG:PLANNER %d cmd(s) initial\n", ring.count());
        }

        for (int i = 0; i < ring.count() && i < 8; i++) {
            const auto& blk = ring.at(i).plan;
            Serial1.printf("DBG:PLAN[%d] dist=%.0f fwd=%d entry=%.0f cruise=%.0f exit=%.0f\n",
                           i, blk.dist, (int)blk.forward,
                           blk.entryVel, blk.cruiseVel, blk.exitVel);
        }

        // ---- Reset faults ----
        g_fault_lag = g_fault_estop = g_fault_estop_gui = g_fault_brownout = false;
        xQueueReset(s_teleQueue);
        s_running   = true;
        s_stopRequested = false;

        char     stopReason[32] = "Completed";
        uint32_t lastVBusMs    = millis();
        uint32_t lastSGMs      = millis();
        uint32_t prevPlanUs    = micros();
        TickType_t xLastWake   = xTaskGetTickCount();
        int      blocksExecuted = 0;

        // ---- Start executing head block ----
        planner.currentVel = 0.0f;
        planner.currentAcc = 0.0f;
        {
            const auto& be = ring.peekHead();
            planner.resetForBlock(be.startPos, be.endPos, be.plan, currentJerkConfig());
            g_target_pos = be.endPos;
            Serial1.printf("DBG:BLOCK[0] start=%.0f end=%.0f entry=%.0f exit=%.0f\n",
                           be.startPos, be.endPos, be.plan.entryVel, be.plan.exitVel);
        }

        // ---- Continuous event loop at 500 Hz ----
        while (s_running) {
            // 0. Controlled stop request (loop_stop or stop command)
            if (s_stopRequested) {
                float curVel   = std::fabs(planner.currentVel);
                float curAccel = ring.isEmpty() ? 5000.0f : ring.peekHead().plan.accel;
                bool  fwd      = (planner.currentVel >= 0.0f);

                planner::injectStopBlock(ring, planner.currentPos, curVel, curAccel, fwd);
                s_stopRequested = false;
                isLoop = false; // no more refilling
                s_plannerState = (s_plannerState == PLANNER_LOOP_RUNNING)
                                 ? PLANNER_LOOP_STOPPING : PLANNER_STOPPING;

                if (!ring.isEmpty()) {
                    const auto& be = ring.peekHead();
                    planner.resetForBlock(planner.currentPos, be.endPos, be.plan, currentJerkConfig());
                    g_target_pos = be.endPos;
                    Serial1.printf("DBG:STOP_INJECT dist=%.0f from vel=%.0f\n",
                                   be.plan.dist, curVel);
                } else {
                    // Already stopped
                    strncpy(stopReason, "Stopped", 31);
                    s_running = false;
                    break;
                }
            }

            // 1. Accept new commands (streaming mode only)
            if (!isLoop) {
                bool newCmds = false;
                while (!ring.isFull()) {
                    if (xQueueReceive(s_motionQueue, &cmd, 0) == pdPASS) {
                        appendCommandToRing(cmd, ring, tailPos);
                        newCmds = true;
                    } else {
                        break;
                    }
                }
                if (newCmds) {
                    planner::incrementalPlan(ring, ring.at(0).plan.entryVel);
                    // Update the executing block's exit velocity in the trajectory
                    // planner so it adjusts braking on the fly. Without this, the
                    // planner would continue targeting exitVel=0 from the initial plan.
                    planner.updateExitVel(ring.at(0).plan.exitVel);
                    Serial1.printf("DBG:REPLAN %d blocks exitVel=%.0f\n",
                                   ring.count(), ring.at(0).plan.exitVel);
                }
            }

            // 2. Generate trajectory point
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

            // 3. VBus brownout check (200 ms)
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

            // 4. Fault handling
            if (g_fault_lag)          { strncpy(stopReason, "Lag Fault",      31); s_running = false; }
            if (g_fault_estop)        { strncpy(stopReason, "E-STOP (SW1)",   31); s_running = false; }
            if (g_fault_estop_gui)    { strncpy(stopReason, "E-STOP (GUI)",   31); s_running = false; }
            if (g_fault_brownout)     { strncpy(stopReason, "Brownout Fault", 31); s_running = false; }
            if (!s_running) break;

            // 5. Block complete?
            if (planner.isComplete()) {
                Serial1.printf("DBG:BLOCK_DONE vel=%.1f pos=%.1f tgt=%.1f\n",
                               planner.currentVel, planner.currentPos, planner.targetPos);
                blocksExecuted++;

                // Send BLOCK_DONE marker (not after the very last block)
                if (s_teleQueue && ring.count() > 1) {
                    TelemetryData bd = {};
                    bd.type = TELEMETRY_BLOCK_DONE;
                    bd.blockIndex  = (uint8_t)(blocksExecuted - 1);
                    bd.totalBlocks = (uint8_t)(blocksExecuted + ring.count() - 1);
                    bd.pos = (long)(planner.currentPos);
                    xQueueSend(s_teleQueue, &bd, pdMS_TO_TICKS(50));
                }

                ring.advanceHead();

                // Emit QUEUE_STATUS so the host knows how many slots are free
                if (s_teleQueue && !isLoop) {
                    TelemetryData qs = {};
                    qs.type = TELEMETRY_QUEUE_STATUS;
                    qs.queueSlots  = (uint8_t)(planner::BLOCK_RING_SIZE - ring.count());
                    qs.plannerState = (uint8_t)s_plannerState;
                    xQueueSend(s_teleQueue, &qs, 0); // non-blocking
                }

                // Loop refill: keep ring full from pattern
                if (isLoop) {
                    int loopLen = s_loopLen;
                    while (!ring.isFull()) {
                        appendCommandToRing(s_loopPattern[loopIdx % loopLen], ring, tailPos);
                        loopIdx++;
                    }
                    // Replan with committed head velocity and loop wrap exit
                    planner::incrementalPlan(ring, ring.at(0).plan.entryVel, wrapVel);
                }

                if (ring.isEmpty()) {
                    strncpy(stopReason, "Completed", 31);
                    break;
                }

                // Start next block — velocity and position carry over seamlessly
                const auto& be = ring.peekHead();
                planner.resetForBlock(planner.currentPos, be.endPos, be.plan, currentJerkConfig());
                g_target_pos = be.endPos;
                Serial1.printf("DBG:BLOCK[%d] start=%.0f end=%.0f entry=%.0f exit=%.0f\n",
                               blocksExecuted, planner.currentPos, be.endPos,
                               be.plan.entryVel, be.plan.exitVel);
            }

            vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
        }

        // ---- Exit: stop trajectory following immediately ----
        s_running = false;
        s_plannerState = PLANNER_IDLE;
        stepgen::halt();
        Serial1.printf("DBG:PLANNER_DONE reason=%s\n", stopReason);

        bool faulted = g_fault_lag || g_fault_estop || g_fault_estop_gui || g_fault_brownout;
        if (!faulted) {
            g_hold_target = g_target_pos;
            g_hold_active = true;
        } else {
            g_hold_target = g_meas_pos;
            g_hold_active = false;
            tmc::disable();
            s_driver_enabled = false;
            Serial1.printf("DBG:FAULT_TMC_DISABLED\n");
        }

        // Route STOP through the telemetry queue so TelemetryTask owns all
        // USBSerial writes — prevents interleaved bytes on CDC TX buffer.
        if (s_teleQueue) {
            TelemetryData stopData = {};
            stopData.type = TELEMETRY_STOP;
            stopData.pos  = (long)(g_meas_pos / counts_to_steps);
            strncpy(stopData.stopReason, stopReason, sizeof(stopData.stopReason) - 1);
            Serial1.printf("DBG:STOP_QUEUED pos=%ld reason=%s\n", stopData.pos, stopReason);
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
            if (homing::isActive()) {
                // Homing owns the step generator — skip all idle/hold logic
                continue;
            }
            if (g_hold_active) {
                // Hold state machine: CORRECTING → SETTLED
                //
                // CORRECTING: PD loop drives corrections when position is
                //   outside the deadband. Step pulses keep TMC2209 at IRUN.
                // SETTLED: position stayed inside deadband for HOLD_SETTLE_MS.
                //   Step pulses stop → TMC2209 detects standstill → drops to
                //   IHOLD automatically, dramatically reducing motor heat.
                const float holdDeadband = g_hold_deadband_counts * counts_to_steps;
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
                        float correction = pd.compute(g_hold_target, 0.0f, 0.0f, measPos, 0.001f);
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
                                  / (velDt > 0.001f ? velDt : 0.1f);
                    holdVelRefEnc = encCounts;
                    holdVelRefMs  = millis();
                }

                if (millis() - lastTeleMs >= holdTeleInterval) {
                    float holdTeleDt = (millis() - lastTeleMs) / 1000.0f;
                    lastTeleMs  = millis();

                    // Short-window measured velocity over the telemetry interval
                    // in encoder counts/s. During CORRECTING this reuses lastTeleEnc
                    // from the motion path for seamless velocity continuity.
                    float holdMVel = (float)(encCounts - lastTeleEnc)
                                     / (holdTeleDt > 0.001f ? holdTeleDt : 0.01f);
                    lastTeleEnc = encCounts; // keep in sync (used by motion path after next move)

                    // Velocity for chart: short-window measured during CORRECTING so
                    // the deceleration tail is shown honestly; 100ms-window (holdVelEst)
                    // during SETTLED where low noise matters more than instant accuracy.
                    float displayVel = (g_hold_state == HOLD_CORRECTING) ? holdMVel : holdVelEst;

                    if (s_teleQueue) {
                        TelemetryData d;
                        d.type      = TELEMETRY_UPDATE;
                        d.timestamp = micros();
                        d.pos       = (long)(stepgen::getStepCount() / counts_to_steps);
                        d.meas      = encCounts;
                        d.target    = (long)(g_hold_target / counts_to_steps);
                        d.lag       = (int)((g_hold_target - measPos) / counts_to_steps);
                        d.vel       = (int)displayVel;  // already enc counts/s
                        d.p_acc     = 0;
                        d.p_dist    = 0;
                        d.sg_result = g_sg_result;
                        d.cs_actual = g_cs_actual_cache;
                        d.pwm_scale = g_pwm_scale_cache;
                        d.mvel      = (int16_t)holdMVel; // already enc counts/s
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
        pd.setAccelFFGain(g_ka);
        pd.setDFilterAlpha(g_d_alpha);

        // --- Consume latest trajectory point (reuse last if buffer empty) ---
        TrajectoryPoint newRef;
        if (trajbuf::pop(newRef)) {
            ref = newRef;
        }

        // --- PD + feedforward velocity command ---
        float correction  = pd.compute(ref.pos, ref.vel, ref.acc, measPos, 0.001f);
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
            float mVel    = (float)(encCounts - lastTeleEnc) / dt_s; // enc counts/s
            lastTeleEnc   = encCounts;

            if (s_teleQueue) {
                TelemetryData d;
                d.type      = TELEMETRY_UPDATE;
                d.timestamp = micros();
                d.pos       = (long)(stepgen::getStepCount() / counts_to_steps);
                d.meas      = encCounts;
                d.target    = (long)(ref.pos / counts_to_steps);
                d.lag       = (int)((ref.pos - measPos) / counts_to_steps);
                d.vel       = (int)(ref.vel / counts_to_steps);
                d.p_acc     = (int)(ref.acc / counts_to_steps);
                d.p_dist    = (int)((g_target_pos - measPos) / counts_to_steps);
                d.sg_result = g_sg_result;
                d.cs_actual = g_cs_actual_cache;
                d.pwm_scale = g_pwm_scale_cache;
                d.mvel      = (int16_t)mVel; // enc counts/s
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

void setAccelFFGain(float ka) {
    g_ka = ka;
}

void setDFilterAlpha(float alpha) {
    g_d_alpha = alpha;
}

void setHoldDeadband(float counts) {
    if (counts < 0.5f) counts = 0.5f;
    if (counts > 20.0f) counts = 20.0f;
    g_hold_deadband_counts = counts;
}
float getHoldDeadband() { return g_hold_deadband_counts; }

void setJerk(float jerkStepsPerSec3) {
    g_jerk = jerkStepsPerSec3;
}

void setJerkRampTime(float rampSeconds) {
    g_jerk_ramp_s = rampSeconds;
    g_jerk = 0.0f;  // clear legacy value so ramp-time takes priority
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

bool startLoop(const MotionCommand* cmds, int n) {
    if (s_running || n < 1 || n > planner::MAX_CHAIN_LEN) return false;
    memcpy(s_loopPattern, cmds, n * sizeof(MotionCommand));
    s_loopLen = n;
    s_plannerState = PLANNER_LOOP_RUNNING;
    // Send a dummy command to wake PlannerTask from its blocking xQueueReceive.
    // PlannerTask checks s_plannerState and enters loop mode.
    MotionCommand wake = {0, 1000.0f, 1000.0f, false, false};
    return xQueueSend(s_motionQueue, &wake, 0) == pdPASS;
}

void stopLoop() {
    s_stopRequested = true;
}

void controlledStop() {
    s_stopRequested = true;
}

PlannerState getPlannerState() { return s_plannerState; }

bool isRunning() { return s_running; }
bool isHoldActive() { return g_hold_active; }
float getHoldTarget() { return g_hold_target; }
uint8_t getHoldState() { return g_hold_state; }

bool isBrownoutFault() { return g_fault_brownout; }
bool isLagFault()      { return g_fault_lag; }
bool isEstopFault()    { return g_fault_estop || g_fault_estop_gui; }

void triggerEstop() {
    g_fault_estop_gui = true;
    stepgen::halt();          // stop step pulses immediately
    g_hold_active = false;    // kill PD hold loop
    tmc::disable();           // de-energize motor coils
    s_driver_enabled = false; // PlannerTask will re-enable on next move
}

void resetPositions() {
    g_meas_pos        = 0.0f;
    g_target_pos      = 0.0f;
    g_hold_target     = 0.0f;
    g_hold_active     = false;
    g_hold_state      = HOLD_CORRECTING;
    g_settle_start_ms = 0;
    g_fault_lag       = false;
    g_fault_estop     = false;
    g_fault_estop_gui = false;
    g_fault_brownout  = false;
}

void reZero() {
    // Zero position state but preserve g_hold_active so the control task
    // continues running and sending UPDATE packets at the new zero.
    // The encoder is already zeroed by the caller (encoder::resetPosition()).
    g_meas_pos        = 0.0f;
    g_target_pos      = 0.0f;
    g_hold_target     = 0.0f;
    g_hold_state      = HOLD_CORRECTING;
    g_settle_start_ms = 0;
}

void sendHomingResult(uint8_t result, uint16_t sgMinFast, uint16_t sgMinSlow,
                      uint16_t sgBaseFast, uint16_t sgBaseSlow,
                      long finalPos, const char* errorMsg) {
    // Packet 0xAE — HOMING_DONE (34 bytes total)
    // [0]     0xAA
    // [1]     0xAE
    // [2]     result (u8)
    // [3-4]   sgMinFast  (uint16 LE)
    // [5-6]   sgMinSlow  (uint16 LE)
    // [7-8]   sgBaseFast (uint16 LE)
    // [9-10]  sgBaseSlow (uint16 LE)
    // [11-14] finalPos   (int32 LE)
    // [15-32] errorMsg   (18 bytes, null-padded)
    // [33]    XOR checksum over bytes [2..32]
    uint8_t buf[34];
    buf[0] = 0xAA;
    buf[1] = 0xAE;
    buf[2] = result;

    uint16_t sgf  = sgMinFast;
    uint16_t sgs  = sgMinSlow;
    uint16_t sgbf = sgBaseFast;
    uint16_t sgbs = sgBaseSlow;
    int32_t  fp   = (int32_t)finalPos;
    memcpy(&buf[3],  &sgf,  2);
    memcpy(&buf[5],  &sgs,  2);
    memcpy(&buf[7],  &sgbf, 2);
    memcpy(&buf[9],  &sgbs, 2);
    memcpy(&buf[11], &fp,   4);

    memset(&buf[15], 0, 18);
    if (errorMsg && errorMsg[0]) {
        strncpy((char*)&buf[15], errorMsg, 17);
        buf[32] = '\0'; // ensure null terminator within field
    }

    uint8_t chk = 0;
    for (int i = 2; i < 33; i++) chk ^= buf[i];
    buf[33] = chk;

    UsbWriteGuard guard;
    if (guard) USBSerial.write(buf, sizeof(buf));
}

} // namespace motion
