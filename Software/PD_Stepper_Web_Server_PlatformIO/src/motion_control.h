#pragma once

#include "telemetry_provider.h"
#include <Arduino.h>

namespace motion {

struct MotionCommand {
    long  distance;      // Relative distance or absolute target in microsteps
    float acceleration;  // Max acceleration  (microsteps/sec²)
    float maxSpeed;      // Max velocity       (microsteps/sec)
    bool  absolute;      // true → distance is an absolute position
    bool  chain;         // true → do not stop/disable at move end; transition to next queued command
};

// Inject the telemetry provider before calling init().
void setTelemetryProvider(TelemetryProvider *provider);

// Set PD feedback gains (replaces the former setPID / PI controller).
//   Kp — proportional gain on phase error
//   Kd — derivative  gain on phase error rate
void setPD(float kp, float kd);

// Set the phase-lead gain Kv. The reference position is advanced by
// Kv * target_velocity to pre-compensate encoder lag at speed.
// Start at 0 and increase in small steps during tuning.
void setPhaseLeadGain(float kv);

// Set the USB-PD configured supply voltage (V). The planner computes a
// brownout threshold of 70% of this value and trips a fault if VBus drops
// below it. Call this from setup() after readSettings().
void setConfiguredVoltage(float volts);

// Set microsteps per full step. Call this whenever the TMC2209 microstep
// setting changes so the motion controller can update its encoder scale.
// Thread-safe (stores to a volatile int32_t read by both tasks).
void setMicrosteps(int microsteps);

// Initialize the motion system: step generator ISR, planner task (Core 0),
// control task (Core 1). Call once from setup() after encoder::init().
void init();

// Enqueue a motion command. Returns false if the queue is full.
// chain=true: do not stop/disable TMC after this move; smoothly transition
// to the next queued command (same direction → velocity continuity;
// opposite direction → zero-velocity handoff with no TMC disable cycle).
bool addCommand(long distance, float acceleration, float maxSpeed,
                bool absolute = false, bool chain = false);

// Returns true while a move is in progress.
bool isRunning();

} // namespace motion
