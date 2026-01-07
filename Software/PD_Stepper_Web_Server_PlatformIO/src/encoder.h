#pragma once

#include <Arduino.h>

namespace encoder {

/**
 * @brief Initialize the I2C bus for the AS5600 encoder.
 */
void init();

/**
 * @brief Launch the independent encoder readout task.
 * @param priority FreeRTOS priority (default 2, higher than motion if needed)
 * @param interval_ms Readout frequency (default 1ms)
 */
void startTask(int priority = 2, int interval_ms = 1);

/**
 * @brief Read the current position from the AS5600 encoder (Internal/Legacy).
 * Handles rollover/revolution counting.
 */
void read();

/**
 * @brief Get the total accumulated counts from the encoder.
 * @return signed long total counts
 */
signed long getTotalCounts();

} // namespace encoder
