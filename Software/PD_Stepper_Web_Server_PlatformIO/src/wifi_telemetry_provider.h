#pragma once
#include "telemetry_provider.h"

/**
 * @brief Stub Wi-Fi WebSocket telemetry provider.
 *
 * This class exists to making swapping back to Wi-Fi telemetry straightforward.
 * Implement the methods below using webserver::broadcastWebSocket() when
 * needed.
 *
 * NOTE: Wi-Fi telemetry is intentionally disabled for now because the
 * TCP/WebSocket stack consumes too much RAM and CPU at high update rates,
 * causing watchdog resets and choppy motor motion.  USB binary telemetry is
 * used instead.
 */
class WifiTelemetryProvider : public TelemetryProvider {
public:
  void init() override {
    // TODO: ensure webserver is already running before calling this.
  }

  void sendTelemetry(const TelemetryData &d) override {
    // TODO: Serialize d to JSON and call webserver::broadcastWebSocket(msg).
  }

  void sendStop(const char *reason, long pos) override {
    // TODO: Broadcast a JSON stop message via WebSocket.
  }
};
