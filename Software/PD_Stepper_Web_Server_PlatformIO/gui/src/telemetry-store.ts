/**
 * Ring-buffer for one move's telemetry data, stored in uPlot-native
 * parallel-array format (index 0 = x/time, indices 1–8 = metrics).
 *
 * All position/velocity values are in encoder counts (4096/rev),
 * independent of the microstep setting.
 *
 * Arrays grow during the move and are capped at MAX_POINTS to bound
 * memory. clear() is called when a new move starts.
 */

import type { TelemetryUpdate } from './protocol';
import { encToDeg } from './units';

const MAX_POINTS = 12000; // ~2 min at 100 Hz
const LAG_WINDOW = 20;   // samples for rolling jitter stdev

/** Per-move derived metrics, computed client-side from the UPDATE stream. */
export interface MoveStats {
  skippedCounts:     number;  // (pos−pos₀) − (meas−meas₀): divergence (encoder counts)
  peakSkippedCounts: number;  // max |relative divergence| over the move (encoder counts)
  peakLagCounts:     number;  // max |lag| over the move (encoder counts)
  meanLagCounts:     number;  // mean |lag| over the move (encoder counts)
  lagJitter:         number;  // rolling σ of last N lag samples (encoder counts)
  effortPct:         number;  // |lag| / max(|vel|, 1) × 100 — PD working hard?
  holdPeakDevCounts: number;  // max |lag| during hold phase (encoder counts)
  // Degree-converted accessors
  peakLagDeg:        number;
  meanLagDeg:        number;
  holdPeakDevDeg:    number;
  lagJitterDeg:      number;
  skippedDeg:        number;
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
  private _csActual:   number[] = [];
  private _pwmScale:   number[] = [];

  private _startTs: number | null = null;
  private _pos0 = 0;   // pos at first packet — baseline for relative divergence
  private _meas0 = 0;  // meas at first packet
  private _lastPkt: TelemetryUpdate | null = null;
  private _peakSkippedCounts = 0;
  private _peakLagCounts = 0;
  private _lagAbsSum = 0;
  private _lagSampleCount = 0;
  private _lagWindow: number[] = [];
  private _holdPhase = false;
  private _holdPeakDevCounts = 0;

  get length(): number { return this._t.length; }

  /** Last measured position (encoder counts). Returns 0 if no packets have been received. */
  get lastMeas(): number { return this._lastPkt?.meas ?? 0; }

  get moveStats(): MoveStats {
    const pkt = this._lastPkt;
    if (!pkt) return {
      skippedCounts: 0, peakSkippedCounts: 0, peakLagCounts: 0, meanLagCounts: 0,
      lagJitter: 0, effortPct: 0, holdPeakDevCounts: 0,
      peakLagDeg: 0, meanLagDeg: 0, holdPeakDevDeg: 0, lagJitterDeg: 0, skippedDeg: 0,
    };
    const skippedCounts = (pkt.pos - pkt.meas) - (this._pos0 - this._meas0);
    const jitter = this._lagJitter();
    const meanLagCounts = this._lagSampleCount > 0
      ? this._lagAbsSum / this._lagSampleCount : 0;
    return {
      skippedCounts,
      peakSkippedCounts: this._peakSkippedCounts,
      peakLagCounts:     this._peakLagCounts,
      meanLagCounts,
      lagJitter:         jitter,
      effortPct:         Math.abs(pkt.lag) / Math.max(Math.abs(pkt.vel), 1) * 100,
      holdPeakDevCounts: this._holdPeakDevCounts,
      peakLagDeg:        encToDeg(this._peakLagCounts),
      meanLagDeg:        encToDeg(meanLagCounts),
      holdPeakDevDeg:    encToDeg(this._holdPeakDevCounts),
      lagJitterDeg:      encToDeg(jitter),
      skippedDeg:        encToDeg(Math.abs(skippedCounts)),
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
    this._peakSkippedCounts = 0;
    this._peakLagCounts = 0;
    this._lagAbsSum = 0;
    this._lagSampleCount = 0;
    this._lagWindow = [];
    this._holdPhase = false;
    this._holdPeakDevCounts = 0;
    this._t.length= this._meas.length = this._target.length =
    this._lag.length = this._vel.length = this._accel.length =
    this._pos.length = this._dist.length = this._stallguard.length =
    this._csActual.length = this._pwmScale.length = 0;
  }

  /** Mark the start of hold phase — peak deviation tracking begins. */
  enterHoldPhase(): void {
    this._holdPhase = true;
    this._holdPeakDevCounts = 0;
  }

  /**
   * Process an UPDATE packet. Always updates stats/peaks.
   * When addToChart is true (default), also appends to chart arrays.
   * Pass false during settled hold to freeze the graph while keeping stats live.
   */
  push(pkt: TelemetryUpdate, addToChart = true): void {
    if (this._startTs === null) {
      this._startTs = pkt.timestamp;
      this._pos0  = pkt.pos;   // capture baseline for relative divergence
      this._meas0 = pkt.meas;
    }

    this._lastPkt = pkt;
    const relSkipped = Math.abs((pkt.pos - pkt.meas) - (this._pos0 - this._meas0));
    if (relSkipped > this._peakSkippedCounts) this._peakSkippedCounts = relSkipped;
    if (Math.abs(pkt.lag) > this._peakLagCounts) this._peakLagCounts = Math.abs(pkt.lag);
    this._lagAbsSum += Math.abs(pkt.lag);
    this._lagSampleCount++;
    if (this._holdPhase && Math.abs(pkt.lag) > this._holdPeakDevCounts) {
      this._holdPeakDevCounts = Math.abs(pkt.lag);
    }
    this._lagWindow.push(pkt.lag);
    if (this._lagWindow.length > LAG_WINDOW) this._lagWindow.shift();

    if (!addToChart) return;

    // unsigned 32-bit delta handles micros() wrap at ~71.6 min
    const t = ((pkt.timestamp - this._startTs) >>> 0) / 1_000_000; // µs → s

    this._t.push(t);
    this._meas.push(pkt.meas);
    this._target.push(pkt.target);
    this._lag.push(pkt.lag);
    this._vel.push(pkt.vel);
    this._accel.push(pkt.accel);
    this._pos.push(pkt.pos);
    this._dist.push(pkt.dist);
    this._stallguard.push(pkt.stallguard);
    this._csActual.push(pkt.csActual);
    this._pwmScale.push(pkt.pwmScale);

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
      this._csActual.shift();
      this._pwmScale.shift();
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
      this._csActual,   // series 9
      this._pwmScale,   // series 10
    ];
  }
}
