#pragma once

#include "telemetry_provider.h"
#include <Arduino.h>

namespace motion {

struct MotionCommand {
  long distance;      // Relative distance or Absolute target in steps
  float acceleration; // Max acceleration in steps/s^2
  float maxSpeed;     // Max velocity in steps/s
  bool absolute;      // True if distance is an absolute target
};

/**
 * @brief Set the telemetry provider before calling init().
 *        The provider must remain valid for the lifetime of the motion task.
 */
void setTelemetryProvider(TelemetryProvider *provider);

/**
 * @brief Set the PID values for the motion controller.
 */
void setPID(float kp, float ki);

/**
 * @brief Initialize the motion control system, including the command queue and
 * Core 1 task.
 */
void init();

/**
 * @brief Add a motion command to the queue.
 * @return true if added successfully, false if queue is full.
 */
bool addCommand(long distance, float acceleration, float maxSpeed,
                bool absolute = false);

/**
 * @brief Get the current status of the motion system.
 */
bool isRunning();

} // namespace motion
