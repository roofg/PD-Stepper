# Copilot Instructions for PD-Stepper

## Project Overview

PD-Stepper is a USB PD powered NEMA 17 stepper motor driver and controller built around an ESP32-S3 microcontroller. The active firmware (`PD_Stepper_Web_Server_PlatformIO`) implements a closed-loop phase-lock PD + feedforward motion controller with hardware step-pulse generation, AS5600 magnetic encoder feedback, and TMC2209 stepper driver. All commanding uses USB CDC serial (JSON in, binary telemetry out). There is no WiFi or WebSocket in the current firmware.

## Build, Test, and Lint

### PD Stepper Firmware (ESP32-S3)

The project uses PlatformIO with the `esp32-s3-devkitc-1` environment.

**Build:**
```
platformio run -e esp32-s3-devkitc-1
```

**Upload to device (specify port to avoid ambiguity):**
```
platformio run -e esp32-s3-devkitc-1 -t upload --upload-port COM3
```

**Monitor AUX debug UART (Serial1, 115200 baud):**
```
platformio device monitor --port COM4 --baud 115200
```
> Connect a USB-UART adapter (e.g. FT232R) to AUX1 (GPIO 14, ESP32 TX) + GND.
> Do NOT connect the UART adapter TX to the ESP32 if VCCIO is 5V — damage risk.

### Python Client Scripts

Python clients live at the project root (same folder as `platformio.ini`), not in `src/`.

**Install Python dependencies:**
```
pip install -r requirements.txt
```

**Run a move and stream live telemetry:**
```
python TriggerMove.py
```

**Other scripts:**
- `AutoTunePD.py` — interactive PD gain tuner
- `AutoTunePID.py` — legacy tuner (kept for reference)
- `ChainMoveTest.py` — sequences of moves
- `ABTest.py` — A/B comparison runs
- `StepperClientPython.py` / `StepperClientUSB.py` — lower-level client wrappers

## Architecture Overview

### Multi-Core Task Layout

```
[Encoder Task — Core 0, 200 Hz]   [Loop Task — Core 1, 10 ms yield]
     ↓ raw counts                       ↓ processSerialCommands()
[Planner Task — Core 0, 500 Hz]        ↓ motion::addCommand()
     ↓ trajectory segments         [System Diagnostics 1 Hz → Serial1]
[Trajectory Ring Buffer (16 slots)]
     ↓ consumed by
[Control Task — Core 1, 1 kHz]
     ↓ PD + feedforward velocity cmd
[Step Generator ISR — hardware timer, up to 40 kHz]
     ↓ STEP/DIR pulses
[TMC2209] → Stepper Motor → Encoder feedback
```

**Core 0:** Encoder polling, Planner task, Arduino `loop()` (serial command handler, system diagnostics)
**Core 1:** Control task (1 kHz PD loop), Step generator ISR

### Key Modules (`src/`)

| File | Role |
|---|---|
| `main.cpp` | Setup, GPIO init, USB CDC + Serial1 init, serial command handler (`processSerialCommands`), Preferences storage |
| `motion_control.cpp/h` | `PlannerTask` (500 Hz trajectory gen) + `ControlTask` (1 kHz PD+FF loop) + shutdown sequence |
| `step_generator.cpp/h` | Hardware timer ISR — generates STEP/DIR pulses deterministically |
| `encoder.cpp/h` | AS5600 magnetic encoder via I2C; `encoder::startTask()` runs at 200 Hz on Core 0 |
| `tmc_driver.cpp/h` | TMC2209 UART config: current, microsteps, StealthChop, StallGuard |
| `pd_controller.h` | Header-only PD + phase-lead struct (`Kp=3.0`, `Kd=0.1`, `Kv=0.0`); used by ControlTask |
| `trajectory_buffer.h` | Header-only SPSC ring buffer (16 segments) between PlannerTask and ControlTask — include ONLY from `motion_control.cpp` |
| `telemetry_data.h` | `TelemetryData` struct shared between motion and provider |
| `telemetry_provider.h` | Abstract `TelemetryProvider` interface (`sendTelemetry`, `sendStop`) |
| `usb_telemetry_provider.h` | Binary USB CDC implementation; `0xAA 0xBB` UPDATE (29 bytes) + `0xAA 0xCC` STOP (38 bytes) |

### GPIO Pin Mapping (defined in `main.cpp`)

- **TMC2209:** EN=21, STEP=5, DIR=6, MS1=1, MS2=2, SPREAD=7, TX=17, RX=18, DIAG=16, INDEX=11
- **Encoder:** I2C on ESP32-S3 default SDA/SCL
- **USB PD (CH224K):** PG=15, CFG1=38, CFG2=48, CFG3=47
- **Monitoring:** VBUS=4, NTC=7, LED1=10, LED2=12, SW1=35, SW2=36, SW3=37
- **AUX UART:** AUX1=14 (ESP32 TX → debug out), AUX2=13 (ESP32 RX)

### Communication Protocol

**Commands (USB CDC, 921600 baud) — JSON, newline-terminated:**

```json
{ "cmd": "move", "distance": 3200, "speed": 12000.0, "accel": 9000, "abs": false }
{ "cmd": "set_pd", "kp": 3.0, "kd": 0.1 }
{ "cmd": "set_phase_lead", "kv": 0.002 }
{ "cmd": "set_current", "value": 80 }
{ "cmd": "set_microsteps", "value": 32 }
{ "cmd": "set_voltage", "value": "20" }
{ "cmd": "set_stall_threshold", "value": 10 }
{ "cmd": "set_standstill_mode", "value": "NORMAL" }
{ "cmd": "save" }
{ "cmd": "get_settings" }
```

**Telemetry (USB CDC, host ← ESP32) — binary only, no text ever:**

| Packet | Header | Size | Notes |
|---|---|---|---|
| UPDATE | `0xAA 0xBB` | 29 bytes | timestamp, pos, meas, target, lag, vel, accel, dist, stallguard + XOR checksum |
| STOP | `0xAA 0xCC` | 38 bytes | int32 final pos + 32-char null-padded reason string |

UPDATE packets sent at 10 Hz during motion. STOP sent once when motion ends.

**Debug text output — Serial1 (AUX UART) ONLY:**
- `Serial1.begin(115200, SERIAL_8N1, AUX2, AUX1)` — RX pin first, TX second
- All `printf`/`println` calls use `Serial1`, never `USBSerial`
- `USBSerial` is used exclusively for `read()` (JSON commands) and `write()` (binary packets)
- **NEVER mix text and binary on USBSerial** — causes binary packet corruption on ESP32-S3 USB CDC

### Libraries (platformio.ini)

```
janelia-arduino/TMC2209@^10.1.1
bblanchon/ArduinoJson@^7.2.1
```
No ESPAsyncWebServer or WiFi libraries.

## Key Conventions

### Code Organization

- **Namespaces:** `motion::`, `encoder::`, `tmc_driver::` — no `webserver::` namespace
- **Headers:** Interface-only declarations in `.h`; implementation in `.cpp`
- **Telemetry:** Swap provider by passing a different `TelemetryProvider*` to `motion::setTelemetryProvider()`
- **Pin definitions:** All `#define` pin constants centralised in `main.cpp`
- **Fixed-point friendly:** Use `int32_t` for step counts and positions; avoid heap allocations in ISR or control task

### Configuration & State

- **Preferences:** Stores `voltage`, `current`, `microsteps`, `stall_threshold`, `standstill_mode`, `boot_count`
- **Startup sequence:** PD trigger pins → GPIO → Encoder → TMC2209 → Serial1 → USBSerial → motion::init()
- **Power Good:** Check PG pin (GPIO 15) before enabling TMC2209 — prevents damage if USB PD unavailable

### Motion Control

- **Phase-lock PD + Feedforward:** `PDController` computes velocity correction; planner feedforward provides bulk of velocity; PD closes the phase error loop
- **Phase-lead compensation:** `Kv * target_velocity` advances reference position to pre-compensate encoder lag at speed; start at 0 and tune upward
- **Trajectory buffer:** SPSC ring (16 slots, ~2–5 ms each); decouples 500 Hz planner from 1 kHz control task
- **Step generator ISR:** Fully deterministic; only reads `step_increment` written atomically by ControlTask
- **Shutdown sequence:** `PLANNER_DONE` → `stepgen::halt()` → 100 ms settle → `tmc::disable()` → `sendStop()`
- **Max following error:** Configurable threshold; exceeding it triggers an emergency stop

### Python Client Conventions

- **Transport:** `pyserial` at 921600 baud — NOT asyncio/websockets
- **DTR stability:** Pass `dsrdtr=True` to `serial.Serial()` to prevent ESP32 reset on port open/close
- **Binary parser:** Sync on `0xAA` magic byte; dispatch on second byte (`0xBB`=UPDATE, `0xCC`=STOP)
- **Non-`0xAA` bytes:** Accumulate as text lines (legacy safety net; no text expected on USB in normal operation)
- **Default port:** COM3, 921600 baud

### Testing and Debugging

- **AUX UART monitor:** Connect FT232R (or similar) to AUX1 (GPIO 14) + GND, 115200 baud, to read all `[SYSTEM]`, `[FW]`, `DBG:` messages
- **USB-UART logic levels:** FT232R RXD input is safe with 3.3V ESP32 TX at any VCCIO. FT232R TXD at 5V **will damage** ESP32 GPIO. For read-only monitoring, leave the adapter TX unconnected.
- **No `USBSerial.flush()`:** On ESP32-S3 USB CDC, `flush()` corrupts the TX buffer — never call it
- **Boot resets:** `pyserial` toggles DTR on open/close; use `dsrdtr=True` to suppress ESP32 resets between Python runs
- **Time column:** Displays `micros()` from ESP32 in milliseconds — continuous within a boot, resets on reboot

## Other Examples in Repository

The repository contains multiple independent example projects:

- **Simple_Button_Control:** Direct GPIO control without encoder feedback
- **Basic_Functionality_Test:** Minimal bringup test
- **ESPHome:** Home Assistant integration (YAML configs for blinds/position control)
- **ESP-NOW:** Wireless peer-to-peer position mirroring between two boards
- **Serial_Control:** UART command interface (see `README.md` for AUX pinout)

## Notes

- **USB PD voltage:** Default 20V; CFG1/CFG2/CFG3 pins select voltage — configure before enabling TMC2209
- **Motor current:** Set via `set_current` command at runtime (0–100% of TMC2209 rated current)
- **Microsteps:** Default 32; change with `set_microsteps` and call `motion::setMicrosteps()` to keep encoder scale in sync
- **Cooling:** TMC2209 gets warm under load — ensure heatsinking
- **Board compatibility:** Firmware assumes ESP32-S3 with PD-Stepper PCB pinout — pin assignments are hardcoded
