#include "motion_control.h"
#include "encoder.h"
#include "tmc_driver.h"
#include "web_server.h"
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

extern String microsteps;

namespace motion {

static QueueHandle_t motionQueue = NULL;
static TaskHandle_t motionTaskHandle = NULL;
static bool running = false;

// S-Curve state variables
static float currentPos = 0; // Use float for internal model
static float currentVel = 0;
static float currentAcc = 0;

void MotionTask(void *pvParameters) {
  MotionCommand cmd;
  uint32_t lastPulseTime = 0;
  uint32_t lastControlTime = 0;
  uint32_t lastTelemetryTime = 0;
  const uint32_t controlInterval = 1000; // 1ms control logic update (micros)
  const uint32_t teleInterval = 100000;  // 100ms telemetry (micros)

  // Initial Sync
  float uSteps = (float)microsteps.toInt();
  if (uSteps < 1)
    uSteps = 32.0f;
  float counts_to_steps = (200.0f * uSteps) / 4096.0f;
  currentPos = (float)encoder::getTotalCounts() * counts_to_steps * -1.0f;

  for (;;) {
    if (xQueueReceive(motionQueue, &cmd, portMAX_DELAY) == pdPASS) {
      running = true;
      tmc::enable();

      uSteps = (float)microsteps.toInt();
      if (uSteps < 1)
        uSteps = 32.0f;
      counts_to_steps = (200.0f * uSteps) / 4096.0f;

      float targetPos = cmd.absolute ? (float)cmd.distance
                                     : (currentPos + (float)cmd.distance);
      float maxV = abs(cmd.maxSpeed);
      float maxA = abs(cmd.acceleration);
      float jerk = maxA * 10.0f;

      bool forward = (targetPos > currentPos);
      tmc::setDirection(forward);

      currentAcc = 0;
      currentVel = 0;
      float correction = 0;
      float lag = 0;
      String stopReason = "Normal Completion"; // Track why we stopped
      lastControlTime = micros();
      lastPulseTime = micros();
      lastTelemetryTime = micros();
      long lastTelePos = encoder::getTotalCounts(); // For meas_vel calc

      while (running) {
        uint32_t now = micros();

        // 1. Control Logic (Update trajectory every 1ms)
        if (now - lastControlTime >= controlInterval) {
          lastControlTime = now;

          long measuredCounts = encoder::getTotalCounts();
          float measuredSteps = (float)measuredCounts * counts_to_steps * -1.0f;
          float distToTarget = abs(targetPos - measuredSteps);
          float stoppingDist = (currentVel * currentVel) / (2.0f * maxA);

          float targetAcc = 0;
          bool overshot = forward ? (measuredSteps > targetPos + 1.0f)
                                  : (measuredSteps < targetPos - 1.0f);

          if (overshot) {
            // Gentle Recovery: If close, don't use full acceleration to fix it.
            float recoveryAcc = (distToTarget < 100.0f) ? (maxA * 0.1f) : maxA;
            targetAcc = forward ? -recoveryAcc : recoveryAcc;
            // Damping: if overshot, don't allow velocity to snap back too hard
            if (forward && currentVel < 0)
              currentVel *= 0.8f;
            if (!forward && currentVel > 0)
              currentVel *= 0.8f;
          } else if (distToTarget < stoppingDist + (abs(currentVel) * 0.02f)) {
            // Decelerate earlier (20ms lookahead instead of 10ms)
            targetAcc = forward ? -maxA : maxA;
          } else if (abs(currentVel) < maxV) {
            targetAcc = forward ? maxA : -maxA;
          }

          // Jerk Limiting
          if (currentAcc < targetAcc) {
            currentAcc += jerk * 0.001f;
            if (currentAcc > targetAcc)
              currentAcc = targetAcc;
          } else if (currentAcc > targetAcc) {
            currentAcc -= jerk * 0.001f;
            if (currentAcc < targetAcc)
              currentAcc = targetAcc;
          }

          currentVel += currentAcc * 0.001f;
          if (currentVel > maxV) {
            currentVel = maxV;
            currentAcc = 0;
          }
          if (currentVel < -maxV) {
            currentVel = -maxV;
            currentAcc = 0;
          }

          // Lag Correction
          // Lag Correction
          lag = currentPos - measuredSteps;
          // No deadband, continuous correction. P-Gain 0.01 for very soft
          // nudges.
          correction = lag * 0.01f;
          if (correction > maxV * 0.2f)
            correction = maxV * 0.2f;
          if (correction < -maxV * 0.2f)
            correction = -maxV * 0.2f;

          // Velocity for Pulse Gen
          // Note: In pulse mode, currentVel is the primary driver

          // Completion check (Relaxed to prevent fighting)
          if (distToTarget < 5.0f && abs(currentVel) < 100.0f) {
            running = false;
            stopReason = "Target Reached";
          }

          // Emergency Stop (insane lag)
          if (abs(lag) > (200.0f * uSteps * 1.5f)) {
            running = false;
            stopReason = "Lag Fault";
          }

          // Hardware Emergency Stop (SW1) - Debounced
          // Check 5 consecutive times to ensure it's not noise
          int pressCount = 0;
          for (int i = 0; i < 5; i++) {
            if (digitalRead(35) == LOW)
              pressCount++;
          }
          if (pressCount == 5) {
            running = false;
            tmc::disable();
            currentVel = 0;
            stopReason = "E-STOP (SW1)";
          }

          // Global Runaway Limit (200k steps ~ 100 revs)
          if (abs(currentPos) > 200000.0f) {
            running = false;
            tmc::disable();
            stopReason = "Runaway Limit";
          }
        }

        // 2. Pulse Generation (High Frequency)
        // Apply correction to the effective velocity for the pulse generator
        // Negative correction to stabilize lag (Negative Feedback)
        // If Lag > 0 (Field ahead), subtract correction to slow down field
        // logic
        float effectiveVel = currentVel - correction;

        // Removed zero-clamp to allow smooth direction reversal if needed

        if (running && abs(effectiveVel) > 1.0f) {
          uint32_t stepInterval = (uint32_t)(1000000.0f / abs(effectiveVel));

          if (now - lastPulseTime >= stepInterval) {
            // Dynamic Direction Switching
            bool movingForward = (effectiveVel > 0);
            tmc::setDirection(movingForward);

            tmc::step();
            lastPulseTime = now;
            currentPos += (movingForward ? 1.0f : -1.0f);
          }
        }

        if (now - lastTelemetryTime >= teleInterval) {
          float dt = (now - lastTelemetryTime) / 1000000.0f;
          lastTelemetryTime = now;

          long currentTelePos = encoder::getTotalCounts();
          float mSteps = (float)currentTelePos * counts_to_steps * -1.0f;

          // Calculate measured velocity (steps/sec)
          float measuredVel = (float)(currentTelePos - lastTelePos) *
                              counts_to_steps * -1.0f / dt;
          lastTelePos = currentTelePos;

          JsonDocument tDoc;
          tDoc["type"] = "telemetry";
          tDoc["pos"] = (long)currentPos;
          tDoc["meas"] = (long)mSteps;
          tDoc["target"] = (long)targetPos;
          tDoc["vel"] = (int)currentVel;
          tDoc["meas_vel"] = (int)measuredVel;
          tDoc["lag"] = (int)(currentPos - mSteps);
          String msg;
          serializeJson(tDoc, msg);
          webserver::broadcastWebSocket(msg);
        }

        // No more jittery vTaskDelay here. The loop runs at full speed.
        // portYIELD() is handled by FreeRTOS as needed for equal priority
        // tasks.
      }

      // Broadcast Stop Reason
      JsonDocument stopDoc;
      stopDoc["type"] = "stop";
      stopDoc["reason"] = stopReason;
      stopDoc["pos"] = (long)currentPos;
      String stopMsg;
      serializeJson(stopDoc, stopMsg);
      webserver::broadcastWebSocket(stopMsg);

      currentVel = 0; // Reset velocity for clean state
      tmc::moveAtVelocity(0);
      currentVel = 0;
      currentAcc = 0;

      // Final telemetry
      JsonDocument fDoc;
      fDoc["type"] = "done";
      fDoc["pos"] = (long)currentPos;
      String m;
      serializeJson(fDoc, m);
      webserver::broadcastWebSocket(m);

      vTaskDelay(pdMS_TO_TICKS(500));
      if (!running) {
        tmc::disable();
        while (xQueuePeek(motionQueue, &cmd, pdMS_TO_TICKS(100)) == pdFALSE) {
          currentPos =
              (float)encoder::getTotalCounts() * counts_to_steps * -1.0f;
        }
      }
    }
  }
}

void init() {
  if (motionQueue == NULL) {
    motionQueue = xQueueCreate(16, sizeof(MotionCommand));
    // Moved back to Core 1 and lowered priority. Core 0 is for WiFi/Radio.
    // Priority 10 is high enough for motion but less than radio tasks.
    // Priority 20 ensures this isn't interrupted by WiFi stack on Core 1
    xTaskCreatePinnedToCore(MotionTask, "MotionTask", 4096, NULL, 20,
                            &motionTaskHandle, 1);
  }
}

bool addCommand(long distance, float acceleration, float maxSpeed,
                bool absolute) {
  if (motionQueue == NULL)
    return false;
  MotionCommand cmd = {distance, acceleration, maxSpeed, absolute};
  return xQueueSend(motionQueue, &cmd, 0) == pdPASS;
}

bool isRunning() { return running; }

} // namespace motion
