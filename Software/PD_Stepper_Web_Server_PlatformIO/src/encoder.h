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
 * @param interval_ms Readout frequency (default 5ms = 200 Hz)
 */
void startTask(int priority = 5, int interval_ms = 5);

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

/**
 * @brief Reset the encoder position to zero.
 * Zeros total_encoder_counts and revolutions under mutex.
 * prev_raw_counts is left at its current value so the next read()
 * computes the correct delta from the current physical position.
 */
void resetPosition();

} // namespace encoder
