#pragma once

#include "planner_core.h"
#include "telemetry_provider.h"
#include <Arduino.h>

namespace motion {

// Re-export planner types into the motion namespace for backward compatibility
using MotionCommand = planner::MotionCommand;
using PlannedBlock  = planner::PlannedBlock;

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

// Acceleration feedforward gain (seconds). Converts planned acceleration
// (µsteps/s²) into a proactive velocity offset (µsteps/s), reducing the
// position error that the PD loop must correct during accel/decel phases.
// Start at 0, increase in steps of 0.0005. Typical range: 0.0005–0.005.
void setAccelFFGain(float ka);

// Set the D-term EMA filter coefficient (0.0 = no filter, 0.8 = ~35 Hz cutoff).
// Higher values attenuate encoder quantization noise on the derivative more
// aggressively; values above ~0.95 make the D term sluggish.
void setDFilterAlpha(float alpha);

// Set the hold position deadband (encoder counts, 0.5–20).
// The PD hold loop only corrects when |error| exceeds this many counts.
// Smaller = tighter hold accuracy; safe minimum ~2 (AS5600 noise floor ≈ 1–2 counts).
// Default 4 = ±33 µm at 34.18 mm/rev.
void  setHoldDeadband(float counts);
float getHoldDeadband();

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

// Planner state machine
enum PlannerState : uint8_t {
    PLANNER_IDLE         = 0,
    PLANNER_RUNNING      = 1,  // streaming/chain execution
    PLANNER_LOOP_RUNNING = 2,  // loop auto-refill active
    PLANNER_LOOP_STOPPING= 3,  // loop deceleration in progress
    PLANNER_STOPPING     = 4,  // streaming controlled stop
};

// Start a loop: store the command pattern and begin continuous execution.
// Commands are executed cyclically with velocity continuity at the wrap boundary.
// Returns false if motion is already in progress or n is invalid.
bool startLoop(const MotionCommand* cmds, int n);

// Stop the current loop: decelerate to zero and enter hold.
void stopLoop();

// Controlled stop: decelerate to zero from current velocity.
// Works in both streaming and loop modes.
void controlledStop();

// Query the planner state machine.
PlannerState getPlannerState();

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
