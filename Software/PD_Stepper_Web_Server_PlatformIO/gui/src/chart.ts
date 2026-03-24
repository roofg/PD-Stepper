/**
 * TelemetryChart — uPlot wrapper for real-time stepper telemetry.
 *
 * Dual y-axis layout:
 *   pos scale (left)  — meas, target, pos (pulse-counted), dist
 *   err scale (right) — lag, vel, accel, stallguard
 *
 * Legend items are click-to-toggle. A ResizeObserver keeps chart width
 * correct when the window is resized.
 *
 * Zoom handling:
 *   Box-select zoom is implemented via hooks.setSelect. When a selection is
 *   committed, setScale('x') is called and _userZoomed is set to true. Each
 *   subsequent setData call re-applies the zoom range so live data does not
 *   reset the view. resetZoom() clears the state (called by btnResetChart
 *   and at the start of a new move via seedYRange).
 *
 * Y-axis seeding:
 *   seedYRange(posMin, posMax, velMax) pre-programs the Y scale range so the
 *   chart does not thrash during the first packets of a move. The seed expands
 *   to fit data but never shrinks. resetZoom() also clears the seed.
 */

import uPlot from 'uplot';
import type { TelemetryStore } from './telemetry-store';

interface SeriesDef {
  label:  string;
  stroke: string;
  scale:  'pos' | 'err';
  show?:  boolean;
}

// Order matches TelemetryStore.toUplotData() — do not reorder without
// updating both files.
const SERIES_DEFS: SeriesDef[] = [
  { label: 'Measured',    stroke: '#22c55e', scale: 'pos'               },
  { label: 'Target',      stroke: '#3b82f6', scale: 'pos'               },
  { label: 'Lag',         stroke: '#eab308', scale: 'err'               },
  { label: 'Velocity',    stroke: '#a855f7', scale: 'err'               },
  { label: 'Accel',       stroke: '#f97316', scale: 'err', show: false  },
  { label: 'Pos (pulse)', stroke: '#6b7280', scale: 'pos', show: false  },
  { label: 'Dist Left',   stroke: '#ec4899', scale: 'pos', show: false  },
  { label: 'StallGuard',  stroke: '#ef4444', scale: 'err', show: false  },
  { label: 'CS Actual',   stroke: '#06b6d4', scale: 'err', show: false  },
  { label: 'PWM Scale',   stroke: '#d946ef', scale: 'err', show: false  },
];

const GRID_STROKE  = '#2a2a2a';
const AXIS_STROKE  = '#888888';
const CHART_HEIGHT = 280;

export class TelemetryChart {
  private chart: uPlot | null = null;
  private ro:    ResizeObserver | null = null;

  // ── Zoom state ────────────────────────────────────────────────────────────
  // When _userZoomed is true, update() re-applies the saved X range after
  // every setData call so live data cannot reset the view.
  private _userZoomed  = false;
  private _zoomedXMin  = 0;
  private _zoomedXMax  = 0;

  // ── Y-axis seed state ─────────────────────────────────────────────────────
  // When _seedActive, the range functions use these as the floor/ceiling and
  // expand (never shrink) as real data arrives. Cleared by resetZoom().
  private _seedActive  = false;
  private _posSeedMin  = 0;
  private _posSeedMax  = 0;
  private _errSeedMin  = 0;
  private _errSeedMax  = 0;

  init(container: HTMLElement): void {
    const width = container.offsetWidth || 640;

    const opts: uPlot.Options = {
      width,
      height: CHART_HEIGHT,
      scales: {
        x:   {},
        pos: {
          range: (_u, dataMin, dataMax) => this._posRange(dataMin, dataMax),
        },
        err: {
          range: (_u, dataMin, dataMax) => this._errRange(dataMin, dataMax),
        },
      },
      hooks: {
        // uPlot draws a selection box on drag but does NOT zoom automatically.
        // We must call setScale ourselves to implement zoom, then track state
        // so update() can re-apply it after each setData.
        setSelect: [
          (u) => {
            if (u.select.width > 0) {
              const xMin = u.posToVal(u.select.left, 'x');
              const xMax = u.posToVal(u.select.left + u.select.width, 'x');
              this._userZoomed = true;
              this._zoomedXMin = Math.min(xMin, xMax);
              this._zoomedXMax = Math.max(xMin, xMax);
              u.setScale('x', { min: this._zoomedXMin, max: this._zoomedXMax });
              // Clear the selection box without re-firing this hook
              u.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false);
            }
          },
        ],
      },
      axes: [
        {
          stroke: AXIS_STROKE,
          grid:   { stroke: GRID_STROKE },
          ticks:  { stroke: GRID_STROKE },
        },
        {
          scale:     'pos',
          stroke:    AXIS_STROKE,
          label:     'Steps',
          labelSize: 14,
          grid:      { stroke: GRID_STROKE },
          ticks:     { stroke: GRID_STROKE },
        },
        {
          scale:     'err',
          side:      1,
          stroke:    AXIS_STROKE,
          label:     'Lag / Vel / Accel',
          labelSize: 14,
          grid:      { show: false },
          ticks:     { stroke: GRID_STROKE },
        },
      ],
      series: [
        {},
        ...SERIES_DEFS.map(s => ({
          label:  s.label,
          stroke: s.stroke,
          scale:  s.scale,
          show:   s.show !== false,
          width:  1.5,
        })),
      ],
      legend: { show: true },
    };

    const emptyData = Array.from(
      { length: SERIES_DEFS.length + 1 },
      () => [] as number[],
    ) as uPlot.AlignedData;

    this.chart = new uPlot(opts, emptyData, container);
    this._bindLegendToggle();
    this._bindResize(container);
  }

  update(store: TelemetryStore): void {
    if (!this.chart) return;
    this.chart.setData(store.toUplotData() as uPlot.AlignedData);
    // setData resets all scales; restore zoom when the user has zoomed in
    if (this._userZoomed) {
      this.chart.setScale('x', { min: this._zoomedXMin, max: this._zoomedXMax });
    }
  }

  /**
   * Pre-programs the Y scale range for the upcoming move to prevent axis
   * thrash as the first packets arrive and data accumulates from zero.
   *
   * The seeded range expands to fit real data but never shrinks mid-move.
   * Also clears any existing zoom so the new move starts with a fresh view.
   *
   * @param posMin  Expected minimum position value (steps)
   * @param posMax  Expected maximum position value (steps)
   * @param velMax  Expected peak velocity (sets ±err scale range)
   */
  seedYRange(posMin: number, posMax: number, velMax: number): void {
    // Guard against NaN/Infinity from invalid or empty form inputs —
    // if any value is non-finite, skip seeding and let the chart auto-scale.
    if (!Number.isFinite(posMin) || !Number.isFinite(posMax) || !Number.isFinite(velMax)) {
      return;
    }
    // New move — clear zoom so the full trace is visible from the start
    this._userZoomed = false;
    this._seedActive = true;
    this._posSeedMin = posMin;
    this._posSeedMax = posMax === posMin ? posMin + 1 : posMax;
    const errBound = Math.abs(velMax) * 1.2 || 1;
    this._errSeedMin = -errBound;
    this._errSeedMax =  errBound;
  }

  /**
   * Reset both zoom and Y-seed back to full-auto behaviour.
   * Call from the "Reset Chart" button handler.
   */
  resetZoom(): void {
    this._userZoomed = false;
    this._seedActive = false;
    // The next setData call (in update()) will auto-rescale from scratch.
  }

  destroy(): void {
    this.ro?.disconnect();
    this.ro = null;
    this.chart?.destroy();
    this.chart = null;
  }

  // ── Private range helpers ─────────────────────────────────────────────────

  /**
   * Auto-range with padding, guarded against empty-data edge cases
   * (uPlot passes Infinity/-Infinity when no data is present).
   */
  private _autoRange(dataMin: number, dataMax: number): [number, number] {
    if (!Number.isFinite(dataMin) || !Number.isFinite(dataMax) || dataMin > dataMax) {
      return [0, 1];
    }
    if (dataMin === dataMax) {
      const pad = Math.abs(dataMin) * 0.1 + 1;
      return [dataMin - pad, dataMax + pad];
    }
    const pad = (dataMax - dataMin) * 0.1;
    return [dataMin - pad, dataMax + pad];
  }

  private _posRange(dataMin: number, dataMax: number): [number, number] {
    if (!this._seedActive) return this._autoRange(dataMin, dataMax);
    let lo = this._posSeedMin;
    let hi = this._posSeedMax;
    if (Number.isFinite(dataMin)) lo = Math.min(lo, dataMin);
    if (Number.isFinite(dataMax)) hi = Math.max(hi, dataMax);
    // Expand never-shrink: commit expanded extremes back to the seed
    this._posSeedMin = lo;
    this._posSeedMax = hi;
    return lo === hi ? [lo - 1, hi + 1] : [lo, hi];
  }

  private _errRange(dataMin: number, dataMax: number): [number, number] {
    if (!this._seedActive) return this._autoRange(dataMin, dataMax);
    let lo = this._errSeedMin;
    let hi = this._errSeedMax;
    if (Number.isFinite(dataMin)) lo = Math.min(lo, dataMin);
    if (Number.isFinite(dataMax)) hi = Math.max(hi, dataMax);
    this._errSeedMin = lo;
    this._errSeedMax = hi;
    return lo === hi ? [lo - 1, hi + 1] : [lo, hi];
  }

  private _bindLegendToggle(): void {
    const legend = this.chart?.root.querySelector('.u-legend') as HTMLElement | null;
    if (!legend || !this.chart) return;

    legend.addEventListener('click', (e: MouseEvent) => {
      const row = (e.target as HTMLElement).closest('.u-series') as HTMLElement | null;
      if (!row || !this.chart) return;

      const rows = Array.from(legend.querySelectorAll('.u-series'));
      const seriesIdx = rows.indexOf(row); // series[0]=Time (hidden), series[1]=Measured, …
      if (seriesIdx > 0) {
        this.chart.setSeries(seriesIdx, { show: !this.chart.series[seriesIdx].show });
      }
    });
  }

  private _bindResize(container: HTMLElement): void {
    this.ro = new ResizeObserver((entries) => {
      const w = entries[0]?.contentRect.width ?? 0;
      if (!this.chart || w === 0) return;
      this.chart.setSize({ width: w, height: CHART_HEIGHT });
    });
    this.ro.observe(container);
  }
}
