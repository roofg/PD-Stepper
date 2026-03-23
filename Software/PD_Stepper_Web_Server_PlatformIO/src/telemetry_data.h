#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <Arduino.h>

enum TelemetryType { TELEMETRY_UPDATE, TELEMETRY_STOP };

struct TelemetryData {
  TelemetryType type;
  char stopReason[32];
  unsigned long timestamp;
  long pos;
  long meas;
  long target;
  int lag;
  int vel;
  int p_acc;
  int p_dist;
  uint16_t sg_result;
  uint8_t  cs_actual;  // TMC CS_ACTUAL (0–31), cached by DiagnosticsTask
  uint8_t  pwm_scale;  // TMC PWM_SCALE (0–255), cached by DiagnosticsTask
  int16_t  mvel;       // measured encoder velocity (steps/s), computed per telemetry tick
};

#endif
