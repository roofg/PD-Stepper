/**
 * Ring-buffer for one move's telemetry data, stored in uPlot-native
 * parallel-array format (index 0 = x/time, indices 1–8 = metrics).
 *
 * Arrays grow during the move and are capped at MAX_POINTS to bound
 * memory. clear() is called when a new move starts.
 */

import type { TelemetryUpdate } from './protocol';

const MAX_POINTS = 2000; // ~3 min at 10 Hz
const LAG_WINDOW = 20;   // samples for rolling jitter stdev

/** Per-move derived metrics, computed client-side from the UPDATE stream. */
export interface MoveStats {
  skippedSteps:     number;  // (pos−pos₀) − (meas−meas₀): divergence relative to move start
  peakSkippedSteps: number;  // max |relative divergence| over the move
  peakLag:          number;  // max |lag| over the move (steps)
  lagJitter:        number;  // rolling σ of last N lag samples (steps)
  effortPct:        number;  // |lag| / max(|vel|, 1) × 100 — PD working hard?
}

export class TelemetryStore {
  // Parallel arrays — index matches uPlot series order defined in chart.ts
  private _t:          number[] = [];
  private _meas:       number[] = [];
  private _target:     number[] = [];
  private _lag:        number[] = [];
  private _vel:        number[] = [];
  private _accel:      number[] = [];
  private _pos:        number[] = [];
  private _dist:       number[] = [];
  private _stallguard: number[] = [];

  private _startTs: number | null = null;
  private _pos0 = 0;   // pos at first packet — baseline for relative divergence
  private _meas0 = 0;  // meas at first packet
  private _lastPkt: TelemetryUpdate | null = null;
  private _peakSkippedSteps = 0;
  private _peakLag = 0;
  private _lagWindow: number[] = [];

  get length(): number { return this._t.length; }

  get moveStats(): MoveStats {
    const pkt = this._lastPkt;
    if (!pkt) return { skippedSteps: 0, peakSkippedSteps: 0, peakLag: 0, lagJitter: 0, effortPct: 0 };
    const skippedSteps = (pkt.pos - pkt.meas) - (this._pos0 - this._meas0);
    return {
      skippedSteps,
      peakSkippedSteps: this._peakSkippedSteps,
      peakLag:          this._peakLag,
      lagJitter:        this._lagJitter(),
      effortPct:        Math.abs(pkt.lag) / Math.max(Math.abs(pkt.vel), 1) * 100,
    };
  }

  private _lagJitter(): number {
    const w = this._lagWindow;
    const n = w.length;
    if (n < 2) return 0;
    const mean = w.reduce((a, b) => a + b, 0) / n;
    const variance = w.reduce((s, x) => s + (x - mean) ** 2, 0) / n;
    return Math.sqrt(variance);
  }

  clear(): void {
    this._startTs = null;
    this._pos0 = 0;
    this._meas0 = 0;
    this._lastPkt = null;
    this._peakSkippedSteps = 0;
    this._peakLag = 0;
    this._lagWindow = [];
    this._t.length = this._meas.length = this._target.length =
    this._lag.length = this._vel.length = this._accel.length =
    this._pos.length = this._dist.length = this._stallguard.length = 0;
  }

  push(pkt: TelemetryUpdate): void {
    if (this._startTs === null) {
      this._startTs = pkt.timestamp;
      this._pos0  = pkt.pos;   // capture baseline for relative divergence
      this._meas0 = pkt.meas;
    }

    const t = (pkt.timestamp - this._startTs) / 1_000_000; // µs → s

    this._lastPkt = pkt;
    const relSkipped = Math.abs((pkt.pos - pkt.meas) - (this._pos0 - this._meas0));
    if (relSkipped > this._peakSkippedSteps) this._peakSkippedSteps = relSkipped;
    if (Math.abs(pkt.lag) > this._peakLag) this._peakLag = Math.abs(pkt.lag);
    this._lagWindow.push(pkt.lag);
    if (this._lagWindow.length > LAG_WINDOW) this._lagWindow.shift();

    this._t.push(t);
    this._meas.push(pkt.meas);
    this._target.push(pkt.target);
    this._lag.push(pkt.lag);
    this._vel.push(pkt.vel);
    this._accel.push(pkt.accel);
    this._pos.push(pkt.pos);
    this._dist.push(pkt.dist);
    this._stallguard.push(pkt.stallguard);

    // Trim oldest point once cap is exceeded
    if (this._t.length > MAX_POINTS) {
      this._t.shift();
      this._meas.shift();
      this._target.shift();
      this._lag.shift();
      this._vel.shift();
      this._accel.shift();
      this._pos.shift();
      this._dist.shift();
      this._stallguard.shift();
    }
  }

  /**
   * Returns data in uPlot AlignedData format:
   *   [xValues, series1, series2, ...]
   *
   * Series order must match the `series` array in chart.ts.
   */
  toUplotData(): number[][] {
    return [
      this._t,          // x
      this._meas,       // series 1
      this._target,     // series 2
      this._lag,        // series 3
      this._vel,        // series 4
      this._accel,      // series 5
      this._pos,        // series 6
      this._dist,       // series 7
      this._stallguard, // series 8
    ];
  }
}
