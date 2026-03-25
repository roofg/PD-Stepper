/**
 * Unit conversion utilities for PD-Stepper telemetry.
 *
 * The firmware sends all position/velocity/acceleration values in
 * **encoder counts** (AS5600: 4096 counts per motor revolution).
 * These helpers convert between encoder counts and human-friendly
 * units (degrees, revolutions, RPM) and back to firmware microstep
 * units for command input.
 */

export const COUNTS_PER_REV     = 4096;
export const FULL_STEPS_PER_REV = 200;
export const DEG_PER_COUNT      = 360 / COUNTS_PER_REV;  // 0.087890625°

// ── Encoder counts → display units ─────────────────────────────────────────

/** Encoder counts → degrees (multi-turn, not wrapped to 0-360). */
export function encToDeg(counts: number): number {
  return counts * 360 / COUNTS_PER_REV;
}

/** Encoder counts → revolutions. */
export function encToRev(counts: number): number {
  return counts / COUNTS_PER_REV;
}

/** Encoder counts/s → RPM. */
export function encPerSecToRPM(cps: number): number {
  return cps * 60 / COUNTS_PER_REV;
}

/** Encoder counts/s² → deg/s². */
export function encPerSec2ToDegPerSec2(cpss: number): number {
  return cpss * 360 / COUNTS_PER_REV;
}

// ── Display units → encoder counts ─────────────────────────────────────────

/** Degrees → encoder counts. */
export function degToEnc(deg: number): number {
  return deg * COUNTS_PER_REV / 360;
}

/** RPM → encoder counts/s. */
export function rpmToEncPerSec(rpm: number): number {
  return rpm * COUNTS_PER_REV / 60;
}

// ── Display units → firmware microstep units (for JSON commands) ────────────

/** Degrees → microsteps (requires current microstep setting). */
export function degToMicrosteps(deg: number, usteps: number): number {
  return Math.round(deg / 360 * FULL_STEPS_PER_REV * usteps);
}

/** RPM → microsteps/s. */
export function rpmToStepsPerSec(rpm: number, usteps: number): number {
  return rpm / 60 * FULL_STEPS_PER_REV * usteps;
}

/** deg/s² → microsteps/s². */
export function degPerSec2ToStepsPerSec2(dps2: number, usteps: number): number {
  return dps2 / 360 * FULL_STEPS_PER_REV * usteps;
}


/** RPM → microsteps/s (alias used for SpreadCycle threshold). */
export function rpmToMicrostepsPerSec(rpm: number, usteps: number): number {
  return rpmToStepsPerSec(rpm, usteps);
}

/** Microsteps/s → RPM (for displaying existing step-based thresholds). */
export function stepsPerSecToRPM(sps: number, usteps: number): number {
  return sps / (FULL_STEPS_PER_REV * usteps) * 60;
}
