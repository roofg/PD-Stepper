#pragma once
#include <Arduino.h>

namespace homing {

struct HomingParams {
    bool     directionCW;   // true = CW, false = CCW
    int      currentPct;    // motor current during homing (e.g. 40%)
    float    speed1;        // fast approach (microsteps/sec)
    float    speed2;        // slow approach (microsteps/sec)
    int      sgThresh1;     // StallGuard threshold for fast pass
    int      sgThresh2;     // StallGuard threshold for slow pass
    float    backoffSteps;  // backoff distance in microsteps
    uint32_t timeoutMs;     // abort if no stall within this time
    // Settings to restore after homing (caller fills from main.cpp statics)
    int      restoreCurrent;     // run current % to restore
    int      restoreSgThresh;    // SG threshold to restore
    bool     restoreStealthchop; // StealthChop state to restore
    bool     restoreCoolstep;    // CoolStep state to restore
};

enum HomingState : uint8_t {
    HOMING_IDLE = 0,
    HOMING_FAST_APPROACH,
    HOMING_BACKOFF,
    HOMING_SLOW_APPROACH,
    HOMING_ZEROING,
    HOMING_DONE,
    HOMING_ERROR
};

// Initialize homing subsystem. Call once from setup() after motion::init().
void init();

// Start homing. Returns false if motion is in progress or already homing.
bool start(const HomingParams& params);

// Current state (readable from any task).
HomingState getState();

// True if homing is active (any state except IDLE, DONE, ERROR).
bool isActive();

// Last error message (empty string if no error).
const char* lastError();

} // namespace homing
