#pragma once
#include <Arduino.h>
#include <stdint.h>

// Hardware timer ISR step generator for TMC2209 STEP/DIR interface.
//
// Uses a Q16.16 fixed-point phase accumulator for velocity-based stepping
// with no float math inside the ISR. The timer fires at ISR_FREQ_HZ (40 kHz)
// giving a 25 µs period and a maximum step rate of 40 kHz (one step per tick).
//
// Typical conversion (control task):
//   phase_increment = velocity_steps_per_sec * 65536 / ISR_FREQ_HZ
//   Set via setVelocity() which performs this conversion from float.
//
// Pulse width: STEP pin is set HIGH during one ISR tick (25 µs) and LOW at
// the start of the next tick, well above the TMC2209's 1.9 µs minimum.

namespace stepgen {

// Timer frequency. Changing this also shifts the max step rate.
static constexpr uint32_t ISR_FREQ_HZ = 40000;

// Q16.16 unity (1.0) — one full step per ISR tick
static constexpr int32_t Q16_ONE = 65536;

// --- Lifecycle ---

// Attach hardware timer ISR. Call once from motion::init() before tasks start.
void init(int step_pin, int dir_pin);

// --- Velocity control (called from Control Task, Core 1) ---

// Set step rate in signed steps/sec. Positive = forward, negative = reverse.
// Converts to Q16.16 phase increment atomically. Safe from any task.
void setVelocity(float velocity_steps_per_sec);

// Force step output to zero immediately (does not stop the timer).
void halt();

// --- Step counter (written by ISR, readable from any task) ---

// Absolute step count since last resetStepCount(). Updated every pulse.
// int32_t reads are atomic on ESP32-S3.
int32_t getStepCount();
void    resetStepCount();

} // namespace stepgen
