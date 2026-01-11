#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

struct TelemetryData {
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
