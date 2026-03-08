#include "step_generator.h"
#include <Arduino.h>
#include <soc/gpio_reg.h>  // GPIO_OUT_W1TS_REG / GPIO_OUT_W1TC_REG

// ISR-accessible data lives in IRAM. All fields are 32-bit aligned so reads
// and writes are atomic on Xtensa LX7 (ESP32-S3) without a mutex.

namespace stepgen {

static hw_timer_t *s_timer = nullptr;

// Precomputed bitmasks for STEP and DIR GPIO registers (set/clear)
// Valid only for GPIO 0–31 (STEP=5, DIR=6 are both in this range).
static uint32_t s_step_mask = 0;
static uint32_t s_dir_mask  = 0;

// Accumulator and signed increment (Q16.16).
// Direction is encoded in the sign of s_phase_inc: positive = forward,
// negative = reverse. This makes setVelocity() a single 32-bit write,
// which is atomic on Xtensa LX7. Eliminates the race where the ISR
// could fire between separate writes of s_forward and s_phase_inc.
static volatile int32_t s_phase_acc = 0;
static volatile int32_t s_phase_inc = 0;  // Q16.16, signed (+ fwd / - rev)

// Pulse state: STEP pin was driven HIGH last tick → drive LOW this tick first.
static volatile bool    s_step_high  = false;

// Step counter (signed; incremented/decremented by direction).
static volatile int32_t s_step_count = 0;

// ---------------------------------------------------------------------------
// ISR — runs at ISR_FREQ_HZ (40 kHz), must complete in << 25 µs
// ---------------------------------------------------------------------------
void IRAM_ATTR timerISR() {
    // Phase 1: end the previous STEP pulse (HIGH→LOW)
    if (s_step_high) {
        REG_WRITE(GPIO_OUT_W1TC_REG, s_step_mask);
        s_step_high = false;
    }

    // Phase 2: accumulate phase; generate a new pulse when we overflow Q16_ONE.
    // Read s_phase_inc once into a local to keep direction/magnitude consistent
    // for this tick (prevents reading a partially-updated value across the tick).
    int32_t inc = s_phase_inc;
    bool fwd = (inc >= 0);
    s_phase_acc += fwd ? inc : -inc;
    if (s_phase_acc >= Q16_ONE) {
        s_phase_acc -= Q16_ONE;

        // Set direction before the STEP rising edge
        if (fwd) {
            REG_WRITE(GPIO_OUT_W1TS_REG, s_dir_mask);
        } else {
            REG_WRITE(GPIO_OUT_W1TC_REG, s_dir_mask);
        }

        // STEP rising edge (LOW will happen at start of next ISR tick)
        REG_WRITE(GPIO_OUT_W1TS_REG, s_step_mask);
        s_step_high = true;

        // Maintain signed step counter
        if (fwd) {
            s_step_count++;
        } else {
            s_step_count--;
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void init(int step_pin, int dir_pin) {
    s_step_mask = 1UL << step_pin;
    s_dir_mask  = 1UL << dir_pin;

    // Arduino-ESP32 v2 timer API:
    //   timerBegin(timer_num, prescaler, count_up)
    //   APB clock = 80 MHz, prescaler = 2 → 40 MHz timer tick (0.025 µs)
    //   timerAlarmWrite(timer, 1000, true) → 1000 ticks × 0.025 µs = 25 µs → 40 kHz
    s_timer = timerBegin(0, 2, true);           // timer 0, prescaler 2, count-up
    timerAttachInterrupt(s_timer, &timerISR, true); // edge-triggered
    timerAlarmWrite(s_timer, 1000, true);       // 1000 ticks = 25 µs = 40 kHz
    timerAlarmEnable(s_timer);
}

void setVelocity(float velocity_steps_per_sec) {
    if (velocity_steps_per_sec > 0.0f) {
        int32_t inc = (int32_t)(velocity_steps_per_sec * (float)Q16_ONE
                                / (float)ISR_FREQ_HZ);
        if (inc > Q16_ONE) inc = Q16_ONE;
        s_phase_inc = inc;   // positive = forward (single atomic write)
    } else if (velocity_steps_per_sec < 0.0f) {
        int32_t inc = (int32_t)(-velocity_steps_per_sec * (float)Q16_ONE
                                 / (float)ISR_FREQ_HZ);
        if (inc > Q16_ONE) inc = Q16_ONE;
        s_phase_inc = -inc;  // negative = reverse (single atomic write)
    } else {
        s_phase_inc = 0;
    }
}

void halt() {
    s_phase_inc = 0;
    // Drive STEP LOW immediately to avoid a lingering HIGH
    REG_WRITE(GPIO_OUT_W1TC_REG, s_step_mask);
    s_step_high = false;
}

int32_t getStepCount()  { return s_step_count; }
void    resetStepCount() { s_step_count = 0; }

} // namespace stepgen
