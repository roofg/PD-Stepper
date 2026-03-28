#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <Arduino.h>

enum TelemetryType { TELEMETRY_UPDATE, TELEMETRY_STOP, TELEMETRY_BLOCK_DONE, TELEMETRY_QUEUE_STATUS };

struct TelemetryData {
  TelemetryType type;
  char stopReason[32];
  unsigned long timestamp;
  long pos;              // pulse-counted position (encoder-count scale)
  long meas;             // measured encoder position (raw encoder counts)
  long target;           // planner reference position (encoder-count scale)
  int lag;               // target - meas (encoder counts)
  int vel;               // reference velocity (encoder counts/s)
  int p_acc;             // planned acceleration (encoder counts/s²)
  int p_dist;            // remaining distance (encoder counts)
  uint16_t sg_result;
  uint8_t  cs_actual;    // TMC CS_ACTUAL (0–31), cached by DiagnosticsTask
  uint8_t  pwm_scale;    // TMC PWM_SCALE (0–255), cached by DiagnosticsTask
  int16_t  mvel;         // measured encoder velocity (encoder counts/s)
  uint8_t  blockIndex;   // BLOCK_DONE: 0-based block index within chain
  uint8_t  totalBlocks;  // BLOCK_DONE: total blocks in chain
  uint8_t  queueSlots;   // QUEUE_STATUS: free slots in block ring buffer
  uint8_t  plannerState; // QUEUE_STATUS: PlannerState enum value
};

#endif
