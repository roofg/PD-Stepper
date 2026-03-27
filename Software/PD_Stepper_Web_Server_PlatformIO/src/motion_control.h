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

// Pre-planned motion block produced by planChain().
// entry/exit velocities are globally optimal (Marlin-style forward+reverse pass).
struct PlannedBlock {
    float dist;      // unsigned distance (steps)
    float entryVel;  // speed at block start (steps/s, ≥ 0)
    float cruiseVel; // maximum speed within block (steps/s)
    float exitVel;   // speed at block end (steps/s, ≥ 0)
    float accel;     // acceleration magnitude (steps/s²)
    bool  forward;   // true = positive direction
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

// Set the D-term EMA filter coefficient (0.0 = no filter, 0.8 = ~35 Hz cutoff).
// Higher values attenuate encoder quantization noise on the derivative more
// aggressively; values above ~0.95 make the D term sluggish.
void setDFilterAlpha(float alpha);

// Set the S-curve jerk limit (µsteps/s³). Legacy interface — prefer setJerkRampTime().
// 0 = auto (equivalent to maxAccel × 100, essentially trapezoidal).
void setJerk(float jerkStepsPerSec3);

// Set the S-curve ramp time (seconds). Jerk is computed as Accel / rampSeconds,
// so jerk auto-scales with accel — one tuning parameter regardless of move profile.
// 0 = auto (~10 ms, essentially trapezoidal). Typical sweet spot: 0.15–0.20 s.
// Calling this clears any legacy absolute jerk value.
void setJerkRampTime(float rampSeconds);

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

// Hold diagnostics — readable from any task
bool isHoldActive();
float getHoldTarget();
uint8_t getHoldState(); // 0 = CORRECTING, 1 = SETTLED

// Fault state accessors — readable from any task (volatile reads)
bool isBrownoutFault();
bool isLagFault();
bool isEstopFault();

// Software emergency stop — equivalent to pressing SW1. Safe to call from any task.
void triggerEstop();

// Reset all position state to zero and clear fault flags.
// Only safe to call when isRunning() is false and no hold is active.
void resetPositions();

// Re-zero the position coordinate system without clearing hold state.
// Call after encoder::resetPosition(): resets g_meas_pos, g_target_pos,
// g_hold_target to 0 but leaves g_hold_active unchanged so the control
// task continues sending UPDATE packets at the new zero.
void reZero();

// Send a HOMING_DONE packet (0xAE) via USBSerial.
// result: 0=ok 1=timeout 2=instant_stall 3=grinding 4=estop
// sgMinFast/sgMinSlow: minimum SG_RESULT seen during each probe (0xFFFF = stage not reached)
// sgBaseFast/sgBaseSlow: average free-running SG over first 5 post-ignore samples
void sendHomingResult(uint8_t result, uint16_t sgMinFast, uint16_t sgMinSlow,
                      uint16_t sgBaseFast, uint16_t sgBaseSlow,
                      long finalPos, const char* errorMsg);

} // namespace motion
