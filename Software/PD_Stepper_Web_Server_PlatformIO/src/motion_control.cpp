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
// Trajectory Planner (S-curve with jerk limiting)
// Generates a smooth reference trajectory (pos/vel/acc) from a MotionCommand.
// ---------------------------------------------------------------------------
class TrajectoryPlanner {
public:
    float currentPos = 0;
    float currentVel = 0;
    float currentAcc = 0;
    float targetPos  = 0;
    float maxV = 0;
    float maxA = 0;
    float jerk = 0;
    // exitVelocity: speed magnitude at move end (0=full stop, >0=chain handoff).
    // moveForward: set once at reset time — direction of this move segment.
    // signedExitVel() returns the correctly-signed target velocity for damping/completion.
    float exitVelocity = 0;
    bool  moveForward  = true;

    float signedExitVel() const { return moveForward ? exitVelocity : -exitVelocity; }

    void reset(float startPos, float trg, float mv, float ma, float exitVel = 0.0f) {
        currentPos    = startPos;
        targetPos     = trg;
        maxV          = fabsf(mv);
        maxA          = fabsf(ma);
        jerk          = maxA * 100.0f; // snap acceleration for responsiveness
        currentVel    = 0;
        currentAcc    = 0;
        moveForward   = (trg > startPos);
        exitVelocity  = fabsf(exitVel);
        if (exitVelocity > maxV) exitVelocity = maxV; // can't exit faster than we travel
    }

    // Chain variant: preserves currentVel/currentAcc for seamless velocity handoff.
    // Call this instead of reset() when transitioning to the next chained move.
    void resetChained(float trg, float mv, float ma, float exitVel = 0.0f) {
        targetPos    = trg;
        maxV         = fabsf(mv);
        maxA         = fabsf(ma);
        jerk         = maxA * 100.0f;
        moveForward  = (trg > currentPos);
        exitVelocity = fabsf(exitVel);
        if (exitVelocity > maxV) exitVelocity = maxV;
        // currentPos, currentVel, currentAcc intentionally preserved
    }

    void update(float dt) {
        float distToTarget = fabsf(targetPos - currentPos);
        bool  forward      = (targetPos > currentPos);

        float targetAcc = 0.0f;

        if (exitVelocity > 0.0f) {
            // ---- Chain handoff mode ----
            // Accelerate to maxV, brake to exitVelocity, then coast through targetPos.
            // isComplete() fires when the planner position crosses targetPos at ~exitVelocity.
            float spd = fabsf(currentVel);
            bool  dirAligned = moveForward ? (currentVel >= 0.0f) : (currentVel <= 0.0f);

            if (!dirAligned) {
                // Moving in the wrong direction — brake unconditionally.
                targetAcc = (currentVel > 0) ? -maxA : maxA;
            } else {
                float excessVel = spd - exitVelocity;
                if (excessVel > 5.0f) {
                    // Above exit velocity: brake if we won't reach exitVelocity by target.
                    // stoppingDist = (v^2 - ev^2)/(2a) = (v-ev)(v+ev)/(2a)
                    float stoppingDist = excessVel * (spd + exitVelocity) / (2.0f * maxA);
                    if (distToTarget < stoppingDist + excessVel * 0.02f) {
                        targetAcc = (currentVel > 0) ? -maxA : maxA; // brake toward exit vel
                    } else if (spd < maxV) {
                        targetAcc = moveForward ? maxA : -maxA;       // accelerate to cruise
                    }
                    // else: cruise at maxV — targetAcc stays 0
                } else if (spd < exitVelocity - 5.0f) {
                    // Below exit velocity (handles start-from-rest and jerk undershoot):
                    // accelerate back up toward exitVelocity.
                    targetAcc = moveForward ? maxA : -maxA;
                }
                // else: within ±5 steps/s of exitVelocity — coast through target.
            }

        } else {
            // ---- Full-stop mode ----
            // Classic trapezoidal / S-curve decelerate to 0 at targetPos.
            float stoppingDist = (currentVel * currentVel) / (2.0f * maxA);

            float approachVel = maxV;
            if (distToTarget < 500.0f) {
                approachVel = distToTarget * 10.0f;
                if (approachVel < 5.0f)  approachVel = 5.0f;
                if (approachVel > maxV)  approachVel = maxV;
            }

            if (distToTarget < 2.0f && fabsf(currentVel) < 20.0f) {
                // Damping zone: servo currentVel toward 0
                targetAcc = -currentVel * 10.0f;
                if (fabsf(targetAcc) > maxA)
                    targetAcc = (targetAcc > 0) ? maxA : -maxA;
            } else if (distToTarget < stoppingDist + fabsf(currentVel) * 0.02f ||
                       fabsf(currentVel) > approachVel) {
                targetAcc = (currentVel > 0) ? -maxA : maxA; // brake toward 0
            } else {
                if (fabsf(currentVel) < approachVel)
                    targetAcc = forward ? maxA : -maxA;       // accelerate to approach vel
            }
        }

        // Apply jerk limit (S-curve smoothing)
        if (currentAcc < targetAcc) {
            currentAcc += jerk * dt;
            if (currentAcc > targetAcc) currentAcc = targetAcc;
        } else if (currentAcc > targetAcc) {
            currentAcc -= jerk * dt;
            if (currentAcc < targetAcc) currentAcc = targetAcc;
        }

        currentVel += currentAcc * dt;
        if (currentVel >  maxV) { currentVel =  maxV; currentAcc = 0; }
        if (currentVel < -maxV) { currentVel = -maxV; currentAcc = 0; }

        currentPos += currentVel * dt;
    }

    bool isComplete() const {
        if (exitVelocity > 0.0f) {
            // Chain mode: fire when the planner position crosses the handoff point at
            // approximately the junction velocity. 100-step/s tolerance covers
            // jerk-induced velocity undershoot (~40 steps/s typical at maxA=9000).
            bool crossed = moveForward ? (currentPos >= targetPos) : (currentPos <= targetPos);
            return crossed && fabsf(currentVel - signedExitVel()) < 100.0f;
        }
        // Full-stop mode: within 2 steps of target, nearly stationary.
        return fabsf(targetPos - currentPos) < 2.0f && fabsf(currentVel) < 20.0f;
    }
};

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
static void PlannerTask(void *) {
    // PlannerTask is not registered with the task WDT (only the Arduino loopTask
    // is registered by default). The delete call was a no-op and is removed.

    MotionCommand cmd;
    TrajectoryPlanner planner;
    float uSteps           = 32.0f;
    float counts_to_steps  = (200.0f * uSteps) / 4096.0f;

    for (;;) {
        // Block until a command arrives
        if (xQueueReceive(s_motionQueue, &cmd, portMAX_DELAY) != pdPASS) continue;

        // ---- Initialise move ----
        uSteps          = (float)g_uSteps_setting;
        if (uSteps < 1) uSteps = 32.0f;
        counts_to_steps = (200.0f * uSteps) / 4096.0f;

        tmc::enable();
        vTaskDelay(pdMS_TO_TICKS(20)); // wait for rails to stabilise

        float startPos = g_meas_pos;  // use encoder as ground truth
        float target   = cmd.absolute
                         ? (float)cmd.distance
                         : (startPos + (float)cmd.distance);
        g_target_pos   = target;

        // Peek at the next queued command (without dequeuing) to compute
        // the junction velocity for this move. This must be done before reset()
        // so the planner decelerates to the correct exit velocity.
        float junctionVel = 0.0f;
        if (cmd.chain) {
            MotionCommand nextCmd;
            if (xQueuePeek(s_motionQueue, &nextCmd, 0) == pdPASS) {
                float nextTarget = nextCmd.absolute
                                   ? (float)nextCmd.distance
                                   : (target + (float)nextCmd.distance);
                // Direction check: if next move is same direction, use junction velocity;
                // if opposite direction, must decelerate to zero (can't reverse without stopping).
                bool currForward = (target > startPos);
                bool nextForward = (nextTarget > target);
                if (currForward == nextForward) {
                    junctionVel = fminf(cmd.maxSpeed, nextCmd.maxSpeed);
                    // Clamp to the safe entry speed for the next segment:
                    // motor must be able to decelerate from junctionVel to a stop
                    // within nextDist. Without this, short segments overshoot.
                    float nextDist     = fabsf(nextTarget - target);
                    float maxSafeEntry = sqrtf(2.0f * nextCmd.acceleration * nextDist);
                    if (junctionVel > maxSafeEntry) junctionVel = maxSafeEntry;
                    // Clamp to what the current move can actually achieve starting from
                    // rest.  If this move is too short to reach junctionVel the isComplete()
                    // velocity check will never fire at targetPos, causing a large overshoot.
                    float maxAchievable = sqrtf(2.0f * cmd.acceleration * fabsf(target - startPos));
                    if (junctionVel > maxAchievable) junctionVel = maxAchievable;
                }
                // else: junctionVel stays 0 — full deceleration required for reversal
            }
            // If queue is empty when we peek: default to junctionVel=0. The move will
            // decelerate to stop normally. If a command arrives later, it starts fresh.
        }

        planner.reset(startPos, target, cmd.maxSpeed, cmd.acceleration, junctionVel);
        trajbuf::clear();

        // Reset faults and start motion
        g_fault_lag = g_fault_estop = g_fault_brownout = false;
        s_running   = true;

        char     stopReason[32] = "Completed"; // fixed buffer — no heap alloc on exit path
        uint32_t lastVBusMs    = millis();
        uint32_t lastSGMs      = millis();
        uint32_t prevPlanUs    = micros(); // track real elapsed time for planner dt

        TickType_t xLastWake   = xTaskGetTickCount();

        while (s_running) {
            // --- 500 Hz planner update with real dt ---
            uint32_t nowUs = micros();
            float dt = (float)(nowUs - prevPlanUs) * 1e-6f;
            if (dt > 0.005f) dt = 0.005f;
            if (dt < 0.0001f) dt = 0.001f; // guard against zero on first tick
            prevPlanUs = nowUs;
            planner.update(dt);

            TrajectoryPoint pt;
            pt.pos = planner.currentPos;
            pt.vel = planner.currentVel;
            pt.acc = planner.currentAcc;
            trajbuf::push(pt);

            // --- VBus brownout check (200 ms) ---
            if (millis() - lastVBusMs >= 200) {
                lastVBusMs = millis();
                float vbus_mv = (float)analogReadMilliVolts(VBUS_PIN);
                float vbus    = (vbus_mv / 1000.0f) / DIV_RATIO;
                if (vbus < g_brownout_threshold_v) {
                    g_fault_brownout = true;
                }
            }

            // --- StallGuard check (200 ms) — UART is safe on Core 0 ---
            if (millis() - lastSGMs >= 200) {
                lastSGMs  = millis();
                g_sg_result = (uint16_t)tmc::getStallGuardResult();
            }

            // --- Aggregate faults → pick stop reason ---
            if (g_fault_lag)      { strncpy(stopReason, "Lag Fault",      31); s_running = false; }
            if (g_fault_estop)    { strncpy(stopReason, "E-STOP (SW1)",   31); s_running = false; }
            if (g_fault_brownout) { strncpy(stopReason, "Brownout Fault", 31); s_running = false; }

            if (!s_running) break; // fault already set

            // --- Completion check ---
            if (planner.isComplete()) {
                if (cmd.chain) {
                    // Try to dequeue the next command for a chained transition.
                    // Chain transitions are based on planner completion only — the encoder
                    // will track the handoff point via PD control.
                    MotionCommand nextCmd;
                    if (xQueueReceive(s_motionQueue, &nextCmd, 0) == pdPASS) {
                        // Compute the junction velocity for the NEW move (after nextCmd)
                        float nextJunctionVel = 0.0f;
                        if (nextCmd.chain) {
                            MotionCommand afterNext;
                            if (xQueuePeek(s_motionQueue, &afterNext, 0) == pdPASS) {
                                float nextTarget = nextCmd.absolute
                                                   ? (float)nextCmd.distance
                                                   : (target + (float)nextCmd.distance);
                                float afterTarget = afterNext.absolute
                                                    ? (float)afterNext.distance
                                                    : (nextTarget + (float)afterNext.distance);
                                bool nextForward  = (nextTarget > target);
                                bool afterForward = (afterTarget > nextTarget);
                                if (nextForward == afterForward) {
                                    nextJunctionVel = fminf(nextCmd.maxSpeed, afterNext.maxSpeed);
                                    float afterDist = fabsf(afterTarget - nextTarget);
                                    // Reserve 2 planner ticks at max junction speed so the motor
                                    // can stop within the next segment even if isComplete() fires
                                    // one tick late (crossing delay ~10 steps at 5000 steps/s).
                                    float safeAfterDist = fmaxf(0.0f, afterDist - nextJunctionVel * 0.004f);
                                    float maxSafeEntry  = sqrtf(2.0f * afterNext.acceleration * safeAfterDist);
                                    if (nextJunctionVel > maxSafeEntry) nextJunctionVel = maxSafeEntry;
                                }
                            }
                        }

                        // Compute next move's absolute target from the CURRENT target (chain end point)
                        float nextTarget = nextCmd.absolute
                                           ? (float)nextCmd.distance
                                           : (target + (float)nextCmd.distance);

                        // Clamp nextJunctionVel by what nextCmd can achieve entering at the
                        // current chain velocity.  Use planner.currentPos as the true start
                        // (resetChained preserves it), not `target` which may be a few steps
                        // behind due to the crossing-detection delay.
                        {
                            float enterVel       = fabsf(planner.currentVel);
                            float nextMoveDist   = fabsf(nextTarget - planner.currentPos);
                            float maxNextAchiev  = sqrtf(enterVel * enterVel
                                                         + 2.0f * nextCmd.acceleration * nextMoveDist);
                            if (nextJunctionVel > maxNextAchiev) nextJunctionVel = maxNextAchiev;
                        }

                        Serial1.printf("DBG:CHAIN_TRANSITION from=%.0f to=%.0f vel=%.0f\n",
                                       target, nextTarget, planner.currentVel);

                        target       = nextTarget;
                        g_target_pos = target;
                        cmd          = nextCmd;

                        // resetChained() preserves currentVel/currentAcc — no velocity discontinuity
                        planner.resetChained(target, cmd.maxSpeed, cmd.acceleration, nextJunctionVel);

                        // Do NOT clear trajbuf — let it drain naturally to avoid starving ControlTask.
                        // The new trajectory points will be pushed from the next planner tick onward.
                        // s_running stays true, TMC stays enabled — chain is seamless.
                        continue; // skip normal completion path
                    }
                    // Queue was empty by the time we tried to dequeue — fall through to normal stop
                }

                // Non-chain or queue-empty: confirm with encoder before declaring done.
                // (Prevents premature stop if the encoder is still catching up.)
                if (fabsf(g_meas_pos - target) < 30.0f) {
                    strncpy(stopReason, "Completed", 31);
                    s_running  = false;
                }
            }

            // Wait for next 2 ms period (500 Hz)
            vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
        }

        // ---- Clean exit (only reached at end of chain or on fault) ----
        Serial1.printf("DBG:PLANNER_DONE reason=%s\n", stopReason);

        stepgen::halt();
        vTaskDelay(pdMS_TO_TICKS(100)); // settle before disabling driver

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
