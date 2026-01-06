#include "encoder.h"
#include <Wire.h>

namespace encoder {

#define AS5600_ADDRESS 0x36

static signed long total_encoder_counts = 0;
static int prev_raw_counts = 0;
static signed long revolutions = 0;

void init() { Wire.begin(); }

void read() {
  int raw_counts = 0;

  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDRESS, 2);

  if (Wire.available() >= 2) {
    raw_counts = Wire.read() << 8 | Wire.read();
  }

  // Handle full revolutions clockwise and anti clockwise
  if (prev_raw_counts > 3000 && raw_counts < 1000) {
    revolutions++;
  } else if (prev_raw_counts < 1000 && raw_counts > 3000) {
    revolutions--;
  }

  // Update current position measured encoder counts
  prev_raw_counts = raw_counts;

  // Update current position considering revolutions
  total_encoder_counts = raw_counts + (4096 * revolutions);
}

signed long getTotalCounts() { return total_encoder_counts; }

} // namespace encoder
