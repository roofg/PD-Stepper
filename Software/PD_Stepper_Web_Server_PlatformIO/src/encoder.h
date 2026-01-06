#pragma once

#include <Arduino.h>

namespace encoder {

/**
 * @brief Initialize the I2C bus for the AS5600 encoder.
 */
void init();

/**
 * @brief Read the current position from the AS5600 encoder.
 * Handles rollover/revolution counting.
 */
void read();

/**
 * @brief Get the total accumulated counts from the encoder.
 * @return signed long total counts
 */
signed long getTotalCounts();

} // namespace encoder
