#include "motion_control.h"
#include "encoder.h"
#include "pd_controller.h"
#include "step_generator.h"
#include "telemetry_provider.h"
#include "tmc_driver.h"
#include "trajectory_buffer.h"
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define STEP_PIN   5
#define DIR_PIN    6
#define SW1_PIN    35
#define VBUS_PIN   4
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

// PD gains — written via setPD() / setPhaseLeadGain() from main task at rest
static float g_kp = 3.0f;
static float g_kd = 0.1f;
static float g_kv = 0.0f;

static QueueHandle_t     s_motionQueue   = nullptr;
static TaskHandle_t      s_plannerHandle = nullptr;
static TaskHandle_t      s_controlHandle = nullptr;
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

    void reset(float startPos, float trg, float mv, float ma) {
        currentPos = startPos;
        targetPos  = trg;
        maxV = fabsf(mv);
        maxA = fabsf(ma);
        jerk = maxA * 100.0f; // snap acceleration for responsiveness
        currentVel = 0;
        currentAcc = 0;
    }

    void update(float dt) {
        float distToTarget = fabsf(targetPos - currentPos);
        float stoppingDist = (currentVel * currentVel) / (2.0f * maxA);
        bool  forward      = (targetPos > currentPos);

        // Scale approach velocity linearly with distance, with a low floor so
        // the motor can fully decelerate to rest in the last few steps without
        // oscillating. A 50 steps/sec floor at 1 step would cause re-acceleration
        // which prevents settling — keep the floor at 5 steps/sec instead.
        float approachVel = maxV;
        if (distToTarget < 500.0f) {
            approachVel = distToTarget * 10.0f;
            if (approachVel < 5.0f)   approachVel = 5.0f;
            if (approachVel > maxV)   approachVel = maxV;
        }

        float targetAcc = 0;
        if (distToTarget < 2.0f && fabsf(currentVel) < 20.0f) {
            // Close enough to target: damp velocity to zero
            targetAcc = -currentVel * 10.0f;
            if (fabsf(targetAcc) > maxA)
                targetAcc = (targetAcc > 0) ? maxA : -maxA;
        } else if (distToTarget < stoppingDist + fabsf(currentVel) * 0.02f ||
                   fabsf(currentVel) > approachVel) {
            // Braking needed
            targetAcc = (currentVel > 0) ? -maxA : maxA;
        } else {
            // Accelerate toward target
            if (fabsf(currentVel) < approachVel)
                targetAcc = forward ? maxA : -maxA;
            // else cruise at approachVel
        }

        // Apply jerk limit (S-curve)
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
        return fabsf(targetPos - currentPos) < 2.0f && fabsf(currentVel) < 20.0f;
    }
};

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
    esp_task_wdt_delete(NULL);

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

        planner.reset(startPos, target, cmd.maxSpeed, cmd.acceleration);
        trajbuf::clear();

        // Reset faults and start motion
        g_fault_lag = g_fault_estop = g_fault_brownout = false;
        s_running   = true;

        String   stopReason    = "Completed";
        uint32_t lastVBusMs    = millis();
        uint32_t lastSGMs      = millis();

        TickType_t xLastWake   = xTaskGetTickCount();

        while (s_running) {
            // --- 500 Hz planner update ---
            uint32_t nowUs = micros();
            planner.update(0.002f); // fixed 2 ms timestep matches loop period

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
                if (vbus < 9.0f) {
                    g_fault_brownout = true;
                }
            }

            // --- StallGuard check (200 ms) — UART is safe on Core 0 ---
            if (millis() - lastSGMs >= 200) {
                lastSGMs  = millis();
                g_sg_result = (uint16_t)tmc::getStallGuardResult();
            }

            // --- Aggregate faults → pick stop reason ---
            if (g_fault_lag)      { stopReason = "Lag Fault";     s_running = false; }
            if (g_fault_estop)    { stopReason = "E-STOP (SW1)";  s_running = false; }
            if (g_fault_brownout) { stopReason = "Brownout Fault"; s_running = false; }

            // --- Completion: planner finished AND encoder near target ---
            if (!s_running) break; // fault already set
            if (planner.isComplete() && fabsf(g_meas_pos - target) < 30.0f) {
                stopReason = "Completed";
                s_running  = false;
            }

            // Wait for next 2 ms period (500 Hz)
            vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
        }

        // ---- Clean exit ----
        stepgen::halt();
        vTaskDelay(pdMS_TO_TICKS(100)); // settle before disabling driver
        tmc::disable();

        if (s_telemetry) {
            s_telemetry->sendStop(stopReason.c_str(), (long)g_meas_pos);
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
    esp_task_wdt_delete(NULL);

    PDController pd;
    TrajectoryPoint ref = {0.0f, 0.0f, 0.0f};

    float uSteps          = 32.0f;
    float counts_to_steps = (200.0f * uSteps) / 4096.0f;

    uint32_t lastTeleMs   = millis();
    long     lastTeleEnc  = 0;

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

        // --- E-Stop (SW1 button — active LOW) ---
        if (digitalRead(SW1_PIN) == LOW) {
            g_fault_estop = true;
            stepgen::halt();
        }

        // --- Lag fault: encoder position has drifted too far from reference ---
        float phase_error = fabsf(ref.pos - measPos);
        if (phase_error > 200.0f * uSteps * 1.5f) {
            g_fault_lag = true;
            stepgen::halt();
        }

        // --- Telemetry at 10 Hz ---
        if (millis() - lastTeleMs >= 100) {
            float dt_s    = (millis() - lastTeleMs) / 1000.0f;
            lastTeleMs    = millis();
            float mVel    = (float)(encCounts - lastTeleEnc)
                            * counts_to_steps / dt_s;
            lastTeleEnc   = encCounts;

            if (s_telemetry) {
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
                d.sg_result = g_sg_result; // written by Planner Task (safe)
                s_telemetry->sendTelemetry(d);
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

void setMicrosteps(int ms) {
    if (ms < 1) ms = 1;
    g_uSteps_setting = (int32_t)ms;
}

void init() {
    s_motionQueue = xQueueCreate(10, sizeof(MotionCommand));

    // Initialise step generator ISR (timer starts immediately but produces no
    // pulses until setVelocity() is called with a non-zero value).
    stepgen::init(STEP_PIN, DIR_PIN);

    tmc::setRunCurrent(80); // 80 % run current for safe high-speed moves

    // Planner Task: Core 0, lower priority — can use UART/ADC safely
    xTaskCreatePinnedToCore(PlannerTask, "PlannerTask", 8192, nullptr,  5,
                            &s_plannerHandle, 0);

    // Control Task: Core 1, high priority — timing-critical real-time loop
    xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, nullptr, 19,
                            &s_controlHandle, 1);
}

bool addCommand(long distance, float acceleration, float maxSpeed, bool absolute) {
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

    MotionCommand cmd = {distance, acceleration, maxSpeed, absolute};
    return xQueueSend(s_motionQueue, &cmd, 0) == pdPASS;
}

bool isRunning() { return s_running; }

} // namespace motion
