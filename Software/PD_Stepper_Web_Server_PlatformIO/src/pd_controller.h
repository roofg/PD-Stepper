#pragma once
#include <Arduino.h>
#include <math.h>

// PD + feedforward + phase-lead controller.
//
// The feedforward term (target_vel) provides the bulk of the velocity command
// from the motion planner. The PD correction adjusts for position error and
// its derivative, replacing the previous PI controller.
//
// Phase-lead compensation (Kv) advances the position reference by a fraction
// of the current target velocity. This pre-compensates for the encoder lag
// that appears at higher speeds, effectively making the commanded position
// "lead" the planner reference by a velocity-proportional amount.
//
// D-term filter (d_alpha): the AS5600 encoder has 4096 discrete counts/rev,
// so measured_pos advances in integer steps. Differentiating those steps
// produces velocity spikes at the encoder tick rate — audible as thumping at
// higher speeds. An EMA low-pass filter on error_rate suppresses this noise
// while preserving the D term's damping at motion-relevant frequencies.
//   d_alpha = 0.0 → no filter (raw derivative, original behaviour)
//   d_alpha = 0.8 → ~35 Hz cutoff, attenuates encoder tick noise ~20×
//   d_alpha = 0.95 → ~8 Hz cutoff, very smooth but D response becomes sluggish
//
// Usage:
//   PDController pd;
//   pd.setGains(kp, kd);
//   pd.setPhaseLeadGain(kv);       // start at 0, tune upward
//   pd.setDFilterAlpha(d_alpha);   // start at 0.8, tune if needed
//   float correction = pd.compute(ref_pos, ref_vel, measured_pos, dt);
//   float velocity_cmd = ref_vel + correction;
//
// The compute() return value is the velocity correction in microsteps/sec.
// A max_correction clamp prevents the feedback from overwhelming the planner.

struct PDController {
    float Kp = 3.0f;
    float Kd = 0.1f;
    float Kv = 0.0f;              // phase-lead gain (microsteps lead per unit velocity)
    float d_alpha = 0.8f;         // EMA filter coefficient for D term (0=off, <1=smoother)
    float max_correction = 5000.0f; // clamp (microsteps/sec)

    float prev_error    = 0.0f;
    float filtered_rate = 0.0f;   // EMA state for D-term filter

    void reset() {
        prev_error    = 0.0f;
        filtered_rate = 0.0f;
    }

    void setGains(float kp, float kd) {
        Kp = kp;
        Kd = kd;
    }

    void setPhaseLeadGain(float kv) { Kv = kv; }
    void setDFilterAlpha(float a)   { d_alpha = a; }

    // Returns velocity correction in microsteps/sec.
    float compute(float target_pos, float target_vel, float measured_pos, float dt) {
        // Phase-lead: advance reference position proportional to current velocity
        float phase_lead     = Kv * target_vel;
        float commanded_pos  = target_pos + phase_lead;
        float error          = commanded_pos - measured_pos;

        // Deadband: suppress noise-driven correction when essentially on target
        if (fabsf(error) < 1.5f) error = 0.0f;

        // Derivative (backward difference, guarded against dt ≈ 0)
        float raw_rate = (dt > 1e-6f) ? (error - prev_error) / dt : 0.0f;
        prev_error = error;

        // EMA low-pass filter: attenuates encoder quantization spikes on D term
        filtered_rate = d_alpha * filtered_rate + (1.0f - d_alpha) * raw_rate;

        float correction = Kp * error + Kd * filtered_rate;

        // Clamp so feedback never exceeds a safe fraction of max speed
        if (correction >  max_correction) correction =  max_correction;
        if (correction < -max_correction) correction = -max_correction;

        return correction;
    }
};
