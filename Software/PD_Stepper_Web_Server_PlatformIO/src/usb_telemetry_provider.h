#pragma once
#include "telemetry_provider.h"
#include <Arduino.h>

/**
 * @brief Binary USB Serial telemetry provider.
 *
 * Wire protocol (little-endian):
 *
 *   Offset  Size  Field
 *   0       1     Header byte 0: 0xAA
 *   1       1     Header byte 1: 0xBB
 *   2       4     timestamp  (uint32_t, microseconds)
 *   6       4     pos        (int32_t, steps, pulse-counted)
 *   10      4     meas       (int32_t, steps, encoder-derived)
 *   14      4     target     (int32_t, steps, planner reference)
 *   18      2     lag        (int16_t, steps)
 *   20      2     vel        (int16_t, steps/s)
 *   22      2     p_acc      (int16_t, steps/s^2)
 *   24      2     p_dist     (int16_t, steps remaining)
 *   26      2     sg_result  (uint16_t, stallguard result)
 *   28      1     checksum   (XOR of bytes 2..27)
 *        Total = 29 bytes per telemetry packet
 *
 * STOP packets:
 *   Offset  Size  Field
 *   0       1     0xAA
 *   1       1     0xCC  (different second byte marks a STOP)
 *   2       4     pos        (int32_t)
 *   6       32    reason     (null-terminated ASCII string)
 *        Total = 38 bytes per stop packet
 */
class UsbTelemetryProvider : public TelemetryProvider {
public:
  void init() override {
    // USBSerial is already started in main.cpp before this is called.
    // No additional setup required.
  }

  void sendTelemetry(const TelemetryData &d) override {
    // Pack header + payload into a stack-local buffer and write in one call
    // to minimise the number of USB transactions.
    uint8_t buf[29];
    buf[0] = 0xAA;
    buf[1] = 0xBB;

    uint32_t ts = (uint32_t)d.timestamp;
    int32_t pos = (int32_t)d.pos;
    int32_t meas = (int32_t)d.meas;
    int32_t tgt = (int32_t)d.target;
    int16_t lag = (int16_t)d.lag;
    int16_t vel = (int16_t)d.vel;
    int16_t acc = (int16_t)d.p_acc;
    int16_t dist = (int16_t)d.p_dist;
    uint16_t sg = (uint16_t)d.sg_result;

    memcpy(&buf[2], &ts, 4);
    memcpy(&buf[6], &pos, 4);
    memcpy(&buf[10], &meas, 4);
    memcpy(&buf[14], &tgt, 4);
    memcpy(&buf[18], &lag, 2);
    memcpy(&buf[20], &vel, 2);
    memcpy(&buf[22], &acc, 2);
    memcpy(&buf[24], &dist, 2);
    memcpy(&buf[26], &sg, 2);

    // Simple XOR checksum over the payload bytes (offsets 2–27)
    uint8_t chk = 0;
    for (int i = 2; i < 28; i++)
      chk ^= buf[i];
    buf[28] = chk;

    USBSerial.write(buf, sizeof(buf));
  }

  void sendStop(const char *reason, long pos) override {
    uint8_t buf[38];
    buf[0] = 0xAA;
    buf[1] = 0xCC; // STOP marker

    int32_t p = (int32_t)pos;
    memcpy(&buf[2], &p, 4);

    // Null-padded reason string
    memset(&buf[6], 0, 32);
    strncpy((char *)&buf[6], reason, 31);

    USBSerial.write(buf, sizeof(buf));
    USBSerial.flush();
  }
};
