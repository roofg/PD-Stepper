/**
 * TelemetryChart — uPlot wrapper for real-time stepper telemetry.
 *
 * Dual y-axis layout:
 *   pos scale (left)  — meas, target, pos (pulse-counted), dist
 *   err scale (right) — lag, vel, accel, stallguard
 *
 * Legend items are click-to-toggle. A ResizeObserver keeps chart width
 * correct when the window is resized.
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

  init(container: HTMLElement): void {
    const width = container.offsetWidth || 640;

    const opts: uPlot.Options = {
      width,
      height: CHART_HEIGHT,
      scales: {
        x:   {},
        pos: { auto: true },
        err: { auto: true },
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
    this.chart?.setData(store.toUplotData() as uPlot.AlignedData);
  }

  destroy(): void {
    this.ro?.disconnect();
    this.ro = null;
    this.chart?.destroy();
    this.chart = null;
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
