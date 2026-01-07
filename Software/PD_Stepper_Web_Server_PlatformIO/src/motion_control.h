#pragma once

#include <Arduino.h>

namespace motion {

struct MotionCommand {
  long distance;      // Relative distance or Absolute target in steps
  float acceleration; // Max acceleration in steps/s^2
  float maxSpeed;     // Max velocity in steps/s
  bool absolute;      // True if distance is an absolute target
};

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
