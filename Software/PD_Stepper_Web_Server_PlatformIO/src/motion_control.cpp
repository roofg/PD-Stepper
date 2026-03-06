#include "motion_control.h"
#include "encoder.h"
#include "tmc_driver.h"
#include "web_server.h"
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

extern String microsteps;

#include "telemetry_data.h"

namespace motion {

static QueueHandle_t motionQueue = NULL;
static QueueHandle_t telemetryQueue = NULL;
static TaskHandle_t motionTaskHandle = NULL;
static bool running = false;
static volatile bool telemetryEnabled = true;
static float globalPos = 0; // Shared state for telemetry
static volatile float globalVelCorrection =
    0; // Shared between 1kHz and Pulse Loop

// Telemetry Task (Low Priority)
void TelemetryTask(void *pvParameters) {
  TelemetryData data;
  JsonDocument tDoc;
  String msg;
  for (;;) {
    if (xQueueReceive(telemetryQueue, &data, portMAX_DELAY) == pdPASS) {
      tDoc.clear();
      if (data.type == TELEMETRY_STOP) {
        tDoc["type"] = "stop";
        tDoc["reason"] = data.stopReason;
        tDoc["pos"] = data.pos;
      } else {
        tDoc["type"] = "telemetry";
        tDoc["time"] = data.timestamp;
        tDoc["pos"] = data.pos;
        tDoc["meas"] = data.meas;
        tDoc["target"] = data.target;
        tDoc["lag"] = data.lag;
        tDoc["vel"] = data.vel;
        tDoc["p_acc"] = data.p_acc;
        tDoc["p_dist"] = data.p_dist;
      }
      msg = "";
      serializeJson(tDoc, msg);
      webserver::broadcastWebSocket(msg);
    }
  }
}

// =================================================================================
// 1. Trajectory Planner (S-Curve)
// =================================================================================
class TrajectoryPlanner {
public:
  float currentPos = 0;
  float currentVel = 0;
  float currentAcc = 0;
  float targetPos = 0;
  float maxV = 0;
  float maxA = 0;
  float jerk = 0;

  void reset(float startPos, float trg, float mv, float ma) {
    currentPos = startPos;
    targetPos = trg;
    maxV = abs(mv);
    maxA = abs(ma);
    jerk = maxA * 100.0f; // Increase Jerk for snappier response (reduce lag)
    currentVel = 0;
    currentAcc = 0;
  }

  void update(float dt) {
    float distToTarget = abs(targetPos - currentPos);
    float stoppingDist = (currentVel * currentVel) / (2.0f * maxA);
    bool forward = (targetPos > currentPos);

    // Dynamic Approach Speed (Prevent Overshoot)
    // When close (500 steps), limit max speed proportionally
    // This effectively lowers 'maxV' as we get closer, preventing
    // the planner from accelerating to full speed after an overshoot
    // turn-around.
    float approachVel = maxV;
    if (distToTarget < 500.0f) {
      approachVel = distToTarget * 10.0f; // 10 steps -> 100 vel (slow!)
      if (approachVel < 50.0f)
        approachVel = 50.0f; // Min crawl speed
      if (approachVel > maxV)
        approachVel = maxV;
    }

    float targetAcc = 0;

    // 1. At Target (deadzone)
    if (distToTarget < 0.5f && abs(currentVel) < 10.0f) {
      targetAcc = -currentVel * 10.0f; // Damp to absolute zero
      if (abs(targetAcc) > maxA) {
        targetAcc = (targetAcc > 0) ? maxA : -maxA;
      }
    }
    // 2. Braking Needed?
    // Check if we overlap the stopping distance OR if we exceed approach limit
    else if (distToTarget < stoppingDist + (abs(currentVel) * 0.02f) ||
             abs(currentVel) > approachVel) {
      // BRAKE: Accel must oppose Velocity
      // Even if velocity is tiny, we must brake if we are over the limit?
      // No, if vel is tiny, braking just stops us.
      if (abs(currentVel) > 1.0f) {
        targetAcc = (currentVel > 0) ? -maxA : maxA;
      } else {
        // If stopped but need to brake? (e.g. overshot and stopped)
        // We need to Reverse. "Braking" logic only works if moving.
        // If stopped, we fall through to Accel logic.
        // But wait, if abs(currentVel) <= approachVel, we go to step 3.
        // Since approachVel >= 50, if vel < 1, we go to step 3.
        // So this block is ONLY for active braking.
        targetAcc = (currentVel > 0) ? -maxA : maxA;
      }
    }
    // 3. Accelerate to Target
    else {
      // We are safe to accelerate, BUT respects the approachVel limit
      // implicitly because if we accelerate past approachVel, the next cycle
      // will Brake. However, to be smoother, we shouldn't accel if at
      // approachVel.
      if (abs(currentVel) < approachVel) {
        targetAcc = forward ? maxA : -maxA;
      } else {
        targetAcc = 0; // Cruise at approach speed
      }
    }

    // Apply Jerk Limit
    if (currentAcc < targetAcc) {
      currentAcc += jerk * dt;
      if (currentAcc > targetAcc)
        currentAcc = targetAcc;
    } else if (currentAcc > targetAcc) {
      currentAcc -= jerk * dt;
      if (currentAcc < targetAcc)
        currentAcc = targetAcc;
    }

    // Integration
    currentVel += currentAcc * dt;

    // Velocity Limits
    if (currentVel > maxV) {
      currentVel = maxV;
      currentAcc = 0;
    }
    if (currentVel < -maxV) {
      currentVel = -maxV;
      currentAcc = 0;
    }

    currentPos += currentVel * dt;
  }

  // Recovery Mode for when the motor is pulled out of position
  // Overrides the S-curve to gently pull back
  void updateRecovery(float measuredPos, float dt) {
    targetPos = measuredPos; // Reset target to where we are? No, we want to go
                             // TO target.
    // Actually, Planner should be "Reference Generator".
    // If we are recovering, we might want to just update currentPos to measured
    // and re-plan? For now, let's keep the planner pure. It generates the
    // "Perfect Move".
  }
};

// =================================================================================
// 2. PID Controller (Velocity Correction)
// =================================================================================
class PIDController {
public:
  float Kp = 8.0f;  // Increased to 8.0 for Servo-Like stiffness
  float Ki = 0.05f; // Increased to 0.05 for fast equalization
  float integrator = 0;
  float maxInteg = 2000.0f; // Max Integral windup (steps/sec correction)

  void reset() { integrator = 0; }

  float compute(float refPos, float measPos, float dt) {
    float error = refPos - measPos;

    // Integral
    integrator += error * dt * Ki * 1000.0f; // Scale factor
    if (integrator > maxInteg)
      integrator = maxInteg;
    if (integrator < -maxInteg)
      integrator = -maxInteg;

    float output = (error * Kp) + integrator;

    // Limit Output
    // Don't let correction exceed 20% of max speed (safety)
    // Hardcoded limit for now
    if (output > 1000.0f)
      output = 1000.0f;
    if (output < -1000.0f)
      output = -1000.0f;

    return output;
  }
};

// =================================================================================
// 3. Step Generator (Pulse Output)
// =================================================================================
class StepGenerator {
public:
  uint32_t lastPulseTime = 0;

  void update(float velocity, uint32_t now) {
    if (abs(velocity) < 1.0f)
      return;

    uint32_t stepInterval = (uint32_t)(1000000.0f / abs(velocity));

    if (now - lastPulseTime >= stepInterval) {
      bool forward = (velocity > 0);
      tmc::setDirection(forward);
      tmc::step();
      lastPulseTime = now;
      globalPos += (forward ? 1.0f : -1.0f); // Update shared state
    }
  }
};

// =================================================================================
// Main Task
// =================================================================================
void MotionTask(void *pvParameters) {
  MotionCommand cmd;
  const uint32_t controlInterval = 1000;
  const uint32_t teleInterval = 100000;

  // Components
  TrajectoryPlanner planner;
  PIDController pid;
  StepGenerator stepGen;

  // Sync Hardware
  float uSteps = (float)microsteps.toInt();
  if (uSteps < 1)
    uSteps = 32.0f;
  float counts_to_steps = (200.0f * uSteps) / 4096.0f;

  // Init State
  long startCounts = encoder::getTotalCounts();
  float startPos = (float)startCounts * counts_to_steps * -1.0f;
  globalPos = startPos;
  planner.currentPos = startPos;

  uint32_t lastControlTime = micros();
  uint32_t lastTelemetryTime = micros();
  long lastTelePos = startCounts;

  for (;;) {
    if (xQueueReceive(motionQueue, &cmd, portMAX_DELAY) == pdPASS) {
      running = true;
      tmc::enable();

      // Disable watchdog for this task while moving to prevent jitter.
      // 1ms vTaskDelay is too long and causes audible "steps" in the timing.
      esp_task_wdt_delete(NULL);

      // Update config
      uSteps = (float)microsteps.toInt();
      if (uSteps < 1)
        uSteps = 32.0f;
      counts_to_steps = (200.0f * uSteps) / 4096.0f;
      float target = cmd.absolute ? (float)cmd.distance
                                  : (globalPos + (float)cmd.distance);

      // Reset Components
      planner.reset(globalPos, target, cmd.maxSpeed, cmd.acceleration);
      pid.reset();
      String stopReason = "Completed";

      // Reset timestamps to avoid huge dt
      lastControlTime = micros();
      lastTelemetryTime = micros();
      stepGen.lastPulseTime = micros();

      while (running) {
        uint32_t now = micros();

        // -------------------------------------------------------
        // 1. Control Loop (1kHz)
        // -------------------------------------------------------
        if (now - lastControlTime >= controlInterval) {
          float dt = (now - lastControlTime) / 1000000.0f;
          lastControlTime = now;

          // A. Update Reference (Planner)
          planner.update(dt);

          // B. Read Feedback
          long encCounts = encoder::getTotalCounts();
          float measPos = (float)encCounts * counts_to_steps * -1.0f;

          // C. PID Correction
          // Error = Plan - Measured
          // Note: planner.currentPos is the Ideal "S-Curve" position
          float velCorrection = pid.compute(planner.currentPos, measPos, dt);
          globalVelCorrection = velCorrection; // Share with pulse loop

          // D. Output
          float commandVel = planner.currentVel + velCorrection;

          // E. Checks
          float dist = abs(target - measPos); // Check against REAL position

          // Completion
          if (dist < 5.0f && abs(commandVel) < 50.0f &&
              abs(planner.currentVel) < 10.0f) {
            running = false;
            stopReason = "Target Reached";
          }

          // E-Stop (Lag > 200 steps)
          if (abs(planner.currentPos - measPos) > (200.0f * uSteps * 1.5f)) {
            running = false;
            stopReason = "Lag Fault";
          }

          // SW1 Stop
          if (digitalRead(35) == LOW) {
            running = false;
            tmc::disable();
            stopReason = "E-STOP (SW1)";
          }
        }

        // -------------------------------------------------------
        // 2. Pulse Generation (Fastest)
        // -------------------------------------------------------
        // Use the planner's velocity + pre-calculated PID correction
        float outputVel = planner.currentVel + globalVelCorrection;

        stepGen.update(outputVel, now);

        // NO YIELD in the high speed loop. Core 1 is for pulses only.

        // -------------------------------------------------------
        // 3. Telemetry (10Hz)
        // -------------------------------------------------------
        if (now - lastTelemetryTime >= teleInterval) {
          float dt = (now - lastTelemetryTime) / 1000000.0f;
          lastTelemetryTime = now;
          long currCounts = encoder::getTotalCounts();
          float mVel =
              (float)(currCounts - lastTelePos) * counts_to_steps * -1.0f / dt;
          lastTelePos = currCounts;

          TelemetryData tData;
          tData.type = TELEMETRY_UPDATE;
          tData.timestamp = now;
          tData.pos = (long)globalPos;
          tData.meas = (long)((float)currCounts * counts_to_steps * -1.0f);
          tData.target = (long)planner.currentPos;
          tData.lag = (int)(planner.currentPos -
                            ((float)currCounts * counts_to_steps * -1.0f));
          tData.vel = (int)planner.currentVel;
          tData.p_acc = (int)planner.currentAcc;
          tData.p_dist = (int)(planner.targetPos - planner.currentPos);

          // Non-blocking send (overwrite if full)
          if (telemetryEnabled) {
            xQueueOverwrite(telemetryQueue, &tData);
          }
        }
      }

      // Clean exit
      tmc::moveAtVelocity(0);

      TelemetryData stopData;
      stopData.type = TELEMETRY_STOP;
      stopData.pos = (long)globalPos;
      strncpy(stopData.stopReason, stopReason.c_str(),
              sizeof(stopData.stopReason));
      stopData.stopReason[sizeof(stopData.stopReason) - 1] = '\0';

      // Send stop message to TelemetryTask to handle the WebSocket broadcast
      // This prevents the high-priority core 1 task from blocking on network
      // logic
      xQueueOverwrite(telemetryQueue, &stopData);

      running = false;

      // Re-enable watchdog for this task now that motion is done.
      esp_task_wdt_add(NULL);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void init() {
  motionQueue = xQueueCreate(10, sizeof(MotionCommand));
  telemetryQueue = xQueueCreate(1, sizeof(TelemetryData));

  // Motion task on Core 1 (Dedicated for pulses)
  xTaskCreatePinnedToCore(MotionTask, "MotionTask", 8192, NULL, 20,
                          &motionTaskHandle, 1);

  // Telemetry task on Core 0 (Higher priority than WebServer/Encoder)
  xTaskCreatePinnedToCore(TelemetryTask, "TeleTask", 4096, NULL, 10, NULL, 0);
}

bool addCommand(long distance, float acceleration, float maxSpeed,
                bool absolute) {
  MotionCommand cmd = {distance, acceleration, maxSpeed, absolute};
  return xQueueSend(motionQueue, &cmd, 0) == pdPASS;
}

bool isRunning() { return running; }

void setTelemetryEnabled(bool enabled) { telemetryEnabled = enabled; }

} // namespace motion
