#pragma once
#include "telemetry_data.h"

/**
 * @brief Abstract interface for telemetry output providers.
 *
 * Implementations can route telemetry to USB Serial, Wi-Fi WebSocket, UDP, etc.
 * Inject the desired provider via motion::setTelemetryProvider() before init().
 */
class TelemetryProvider {
public:
  virtual void init() = 0;
  virtual void sendTelemetry(const TelemetryData &data) = 0;
  virtual void sendStop(const char *reason, long pos) = 0;
  virtual ~TelemetryProvider() = default;
};
