#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

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
};

#endif
