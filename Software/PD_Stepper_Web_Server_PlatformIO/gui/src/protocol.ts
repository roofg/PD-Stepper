/**
 * Binary protocol parser for PD-Stepper USB CDC telemetry.
 *
 * Wire format (little-endian):
 *   UPDATE   0xAA 0xBB  33 bytes total  (mvel added at offset 30; checksum at 32)
 *   STOP     0xAA 0xCC  38 bytes total
 *   SETTINGS 0xAA 0xEE  19 bytes total
 *   STATUS   0xAA 0xDD  20 bytes total
 */

const SYNC       = 0xaa;
const UPDATE_T   = 0xbb;
const STOP_T     = 0xcc;
const SETTINGS_T = 0xee;
const STATUS_T   = 0xdd;

const UPDATE_LEN   = 33;
const STOP_LEN     = 38;
const SETTINGS_LEN = 19;
const STATUS_LEN   = 20;

export interface TelemetryUpdate {
  type: 'update';
  timestamp: number;   // uint32, µs
  pos:       number;   // int32, pulse-counted steps
  meas:      number;   // int32, encoder-derived steps
  target:    number;   // int32, planner reference steps
  lag:       number;   // int16, target - meas
  vel:       number;   // int16, steps/s
  accel:     number;   // int16, steps/s²
  dist:      number;   // int16, remaining steps
  stallguard: number;  // uint16, TMC2209 StallGuard result
  csActual:  number;   // uint8,  TMC current scale 0–31
  pwmScale:  number;   // uint8,  TMC PWM duty 0–255
  mvel:      number;   // int16,  measured encoder velocity (steps/s)
}

export interface StopPacket {
  type:   'stop';
  pos:    number;  // int32, final position
  reason: string;  // up to 32-char ASCII
}

export type Packet = TelemetryUpdate | StopPacket;

/** Settings snapshot from 0xAA 0xEE packet (19 bytes). */
export interface SettingsPacket {
  voltage:        number;  // V (5/9/12/15/20)
  current:        number;  // run current %
  holdCurrent:    number;  // hold current %
  holdDelay:      number;  // hold delay %
  microsteps:     number;  // 1–256
  stallThreshold: number;  // 0–255
  standstillMode: number;  // 0=NORMAL 1=FREEWHEELING 2=BRAKING 3=STRONG_BRAKING
  stealthchop:    boolean;
  coolstep:       boolean;
  kp:             number;  // (kpInt / 1000)
  kd:             number;  // (kdInt / 10000)
  kv:             number;  // (kvInt / 100000)
}

/** Driver status snapshot from 0xAA 0xDD packet (20 bytes). */
export interface StatusPacket {
  vbusMv:           number;   // VBUS in mV
  // flags_a (byte 4)
  pgOk:             boolean;  // PG pin OK (active-low on CH224K)
  otWarn:           boolean;
  otShutdown:       boolean;
  faultLag:         boolean;
  faultBrownout:    boolean;
  holdActive:       boolean;
  isRunning:        boolean;
  // flags_b (byte 5)
  stealthchopActive: boolean;
  standstill:       boolean;
  shortGndA:        boolean;
  shortGndB:        boolean;
  openLoadA:        boolean;
  openLoadB:        boolean;
  holdSettled:      boolean;
  // measurements
  csActual:         number;   // 0–31
  pwmScale:         number;   // 0–255
  tstep:            number;   // inter-step duration (TMC units)
  sgResult:         number;   // StallGuard 0–1023
  freeHeapKb:       number;
  ctrlHwm:          number;   // ControlTask stack high-water mark (bytes)
  bootCount:        number;
  resetReason:      number;   // esp_reset_reason_t value
}

/** USB link diagnostics — tracked at the parser layer. Reset on each new connection. */
export interface LinkStats {
  updateCount:    number;  // valid UPDATE packets received
  stopCount:      number;  // valid STOP packets received
  checksumErrors: number;  // UPDATE packets with bad XOR checksum
  resyncEvents:   number;  // times parser lost byte-sync and had to rescan
  gapsOver150ms:  number;  // UPDATE inter-packet gaps > 150ms (should be ~100ms)
  maxGapMs:       number;  // largest inter-packet gap seen (ms)
  bytesProcessed: number;  // total bytes fed into the parser
}

export class PacketParser {
  private buf: number[] = [];
  onPacket:   ((pkt: Packet) => void) | null = null;
  onSettings: ((pkt: SettingsPacket) => void) | null = null;
  onStatus:   ((pkt: StatusPacket)   => void) | null = null;

  private _stats: LinkStats = this._zeroStats();
  private _lastUpdateWallMs = 0;
  private _wasDiscarding = false;

  private _zeroStats(): LinkStats {
    return {
      updateCount: 0, stopCount: 0, checksumErrors: 0,
      resyncEvents: 0, gapsOver150ms: 0, maxGapMs: 0, bytesProcessed: 0,
    };
  }

  get stats(): Readonly<LinkStats> { return this._stats; }

  /** Clear buffered bytes — call on each new connection to avoid stale-byte corruption. */
  reset(): void {
    this.buf = [];
  }

  /** Reset all link statistics counters — call on each new connection. */
  resetStats(): void {
    this._stats = this._zeroStats();
    this._lastUpdateWallMs = 0;
    this._wasDiscarding = false;
  }

  feed(chunk: Uint8Array): void {
    this._stats.bytesProcessed += chunk.length;
    for (let i = 0; i < chunk.length; i++) this.buf.push(chunk[i]);
    this._parse();
  }

  private _parse(): void {
    while (this.buf.length >= 2) {
      if (this.buf[0] !== SYNC) {
        if (!this._wasDiscarding) { this._stats.resyncEvents++; this._wasDiscarding = true; }
        this.buf.shift();
        continue;
      }

      const type = this.buf[1];

      if (type === UPDATE_T) {
        if (this.buf.length < UPDATE_LEN) return;

        // Validate XOR checksum over bytes 2..31 BEFORE consuming.
        // Splicing first would irrecoverably lose any valid 0xAA byte
        // embedded in the discarded window (common in int32 position fields).
        let chk = 0;
        for (let i = 2; i < 32; i++) chk ^= this.buf[i];
        if (chk !== this.buf[32]) {
          this._stats.checksumErrors++;
          if (!this._wasDiscarding) { this._stats.resyncEvents++; this._wasDiscarding = true; }
          this.buf.shift(); // discard false 0xAA sync, rescan from next byte
          continue;
        }

        this._wasDiscarding = false;
        const now = Date.now();
        if (this._lastUpdateWallMs > 0) {
          const gap = now - this._lastUpdateWallMs;
          if (gap > this._stats.maxGapMs) this._stats.maxGapMs = gap;
          if (gap > 25) this._stats.gapsOver150ms++;  // threshold: 2.5× 10ms interval
        }
        this._lastUpdateWallMs = now;
        this._stats.updateCount++;

        const pkt = this.buf.splice(0, UPDATE_LEN); // checksum OK — consume
        const dv = new DataView(new Uint8Array(pkt).buffer);
        this.onPacket?.({
          type:       'update',
          timestamp:  dv.getUint32(2, true),
          pos:        dv.getInt32(6, true),
          meas:       dv.getInt32(10, true),
          target:     dv.getInt32(14, true),
          lag:        dv.getInt16(18, true),
          vel:        dv.getInt16(20, true),
          accel:      dv.getInt16(22, true),
          dist:       dv.getInt16(24, true),
          stallguard: dv.getUint16(26, true),
          csActual:   dv.getUint8(28),
          pwmScale:   dv.getUint8(29),
          mvel:       dv.getInt16(30, true),
        });

      } else if (type === STOP_T) {
        if (this.buf.length < STOP_LEN) return;
        this._wasDiscarding = false;
        this._stats.stopCount++;
        const pkt = this.buf.splice(0, STOP_LEN);
        const dv  = new DataView(new Uint8Array(pkt).buffer);
        const pos = dv.getInt32(2, true);
        const raw = new Uint8Array(pkt.slice(6, 38));
        const end = raw.indexOf(0);
        const reason = new TextDecoder().decode(raw.slice(0, end === -1 ? 32 : end));
        this.onPacket?.({ type: 'stop', pos, reason });

      } else if (type === SETTINGS_T) {
        if (this.buf.length < SETTINGS_LEN) return;
        // Validate XOR checksum over bytes [2..17]
        let cs = 0;
        for (let i = 2; i < 18; i++) cs ^= this.buf[i];
        if (cs !== this.buf[18]) {
          this._stats.checksumErrors++;
          if (!this._wasDiscarding) { this._stats.resyncEvents++; this._wasDiscarding = true; }
          this.buf.shift();
          continue;
        }
        this._wasDiscarding = false;
        const b = this.buf.splice(0, SETTINGS_LEN);
        const settings: SettingsPacket = {
          voltage:        b[2],
          current:        b[3],
          holdCurrent:    b[4],
          holdDelay:      b[5],
          microsteps:     b[6] | (b[7] << 8),
          stallThreshold: b[8],
          standstillMode: b[9],
          stealthchop:    b[10] !== 0,
          coolstep:       b[11] !== 0,
          kp:             (b[12] | (b[13] << 8)) / 1000,
          kd:             (b[14] | (b[15] << 8)) / 10000,
          kv:             (b[16] | (b[17] << 8)) / 100000,
        };
        this.onSettings?.(settings);

      } else if (type === STATUS_T) {
        if (this.buf.length < STATUS_LEN) return;
        // Validate XOR checksum over bytes [2..18]
        let cs = 0;
        for (let i = 2; i < 19; i++) cs ^= this.buf[i];
        if (cs !== this.buf[19]) {
          this._stats.checksumErrors++;
          if (!this._wasDiscarding) { this._stats.resyncEvents++; this._wasDiscarding = true; }
          this.buf.shift();
          continue;
        }
        this._wasDiscarding = false;
        const b = this.buf.splice(0, STATUS_LEN);
        const status: StatusPacket = {
          vbusMv:            b[2] | (b[3] << 8),
          pgOk:              (b[4] & 0x01) !== 0,
          otWarn:            (b[4] & 0x02) !== 0,
          otShutdown:        (b[4] & 0x04) !== 0,
          faultLag:          (b[4] & 0x08) !== 0,
          faultBrownout:     (b[4] & 0x20) !== 0,
          holdActive:        (b[4] & 0x40) !== 0,
          isRunning:         (b[4] & 0x80) !== 0,
          stealthchopActive: (b[5] & 0x01) !== 0,
          standstill:        (b[5] & 0x02) !== 0,
          shortGndA:         (b[5] & 0x08) !== 0,
          shortGndB:         (b[5] & 0x10) !== 0,
          openLoadA:         (b[5] & 0x20) !== 0,
          openLoadB:         (b[5] & 0x40) !== 0,
          holdSettled:       (b[5] & 0x80) !== 0,
          csActual:          b[6],
          pwmScale:          b[7],
          tstep:             b[8] | (b[9] << 8),
          sgResult:          b[10] | (b[11] << 8),
          freeHeapKb:        b[12] | (b[13] << 8),
          ctrlHwm:           b[14] | (b[15] << 8),
          bootCount:         b[16] | (b[17] << 8),
          resetReason:       b[18],
        };
        this.onStatus?.(status);

      } else {
        if (!this._wasDiscarding) { this._stats.resyncEvents++; this._wasDiscarding = true; }
        this.buf.shift(); // unknown second byte — skip sync and rescan
      }
    }
  }
}

export function moveCommand(
  distance: number,
  speed:    number,
  accel:    number,
  abs = false,
): string {
  return JSON.stringify({ cmd: 'move', distance, speed, accel, abs }) + '\n';
}

export function setCurrentCommand(percent: number): string {
  return JSON.stringify({ cmd: 'set_current', value: percent }) + '\n';
}
