---
name: build-and-flash
description: "Guide for building the PD-Stepper firmware with PlatformIO and flashing it to the device via COM3 (ESP32-S3 USB CDC). Use this when asked to build, compile, upload, flash, or deploy firmware to the PD-Stepper board."
---

The PD-Stepper firmware targets the `esp32-s3-devkitc-1` PlatformIO environment.
Project root (contains `platformio.ini`): `Software/PD_Stepper_Web_Server_PlatformIO/`

## Port assignments (this machine)

- **COM3** — ESP32-S3 USB CDC (VID 303A / PID 1001) — used for flash upload and binary telemetry
- **COM4** — FT232RL AUX debug UART (VID 0403 / PID 6001) — used for `Serial1` debug output

## Build only

```
cd Software/PD_Stepper_Web_Server_PlatformIO
platformio run -e esp32-s3-devkitc-1
```

Expected output on success:
```
[SUCCESS] Took ~2–30 s
RAM:   [=         ]   6.4% (used 20836 bytes from 327680 bytes)
Flash: [=         ]  10.4% (used 346125 bytes from 3342336 bytes)
```

If RAM or Flash usage looks unexpectedly high after a change, check for accidental heap allocations in ISR or control-task code.

## Flash to device

Always specify `--upload-port` to avoid accidentally flashing the wrong device:

```
cd Software/PD_Stepper_Web_Server_PlatformIO
platformio run -e esp32-s3-devkitc-1 -t upload --upload-port COM3
```

The ESP32-S3 enters bootloader automatically via USB CDC DTR/RTS toggle — no physical boot button needed.

## Verify after flash (programmatic)

After flashing, the board reboots immediately. The `Setup complete` boot message is often missed because the board resets before the monitoring script opens the port — this is normal. The reliable success indicator is `[SYSTEM]` appearing within ~2 s of connecting.

Use `pyserial` on COM4 to confirm a clean boot via the AUX debug UART (Serial1). `pyserial` is available in the project venv.

```python
import serial, time, re

PORT = "COM4"
BAUD = 115200
TIMEOUT_SEC = 8  # allow time for reboot + 1 Hz [SYSTEM] tick

s = serial.Serial(PORT, BAUD, timeout=3, dsrdtr=True)
time.sleep(0.2)

deadline = time.time() + TIMEOUT_SEC
results = {"system": False, "pg_ok": False, "vbus": None}

while time.time() < deadline:
    line = s.readline().decode("utf-8", errors="replace").strip()
    if not line:
        continue
    print(line)
    if "[SYSTEM]" in line:
        results["system"] = True
        m = re.search(r"VBus:\s*([\d.]+)V.*PG:\s*(\w+)", line)
        if m:
            results["vbus"] = float(m.group(1))
            results["pg_ok"] = m.group(2) == "OK"
        break

s.close()

print("\n--- POST-FLASH VERIFICATION ---")
print(f"[SYSTEM] seen : {'OK' if results['system'] else 'FAIL - firmware not running'}")
print(f"PG: OK        : {'OK' if results['pg_ok'] else 'FAIL (TMC2209 disabled)'}")
print(f"VBus          : {results['vbus']}V" if results["vbus"] is not None else "VBus          : not detected")
```

Key parameters:
- `dsrdtr=True` — prevents spurious ESP32 reset when the Python script opens the port
- `TIMEOUT_SEC = 8` — covers reboot (~1–2 s) plus up to one full [SYSTEM] tick (1 Hz)
- COM4 must be free — close PlatformIO monitor if open before running

## Boot sequence (expected on Serial1)

```
--- SYSTEM BOOT #N ---
Reset Reason: Software Reset
[SERIAL] Ready
Setup complete
[SYSTEM] VBus: 20.05V, PG: OK, Core: 1
```

`Reset Reason: Software Reset` is expected after a flash upload. `Power-on` after a cold start. `External Pin` after a manual RST press — all three are normal.

Crash-indicating reasons: `Software Panic`, `Interrupt Watchdog`, `Task Watchdog`, `Brownout`. If boot count is incrementing rapidly, a crash loop is occurring — check these.

## Pass / fail criteria

| Signal | Pass | Fail |
|---|---|---|
| Build exit code | 0 | Non-zero — check compiler errors |
| Upload exit code | 0 | Non-zero — wrong port, board not in bootloader, USB cable issue |
| `[SYSTEM]` on Serial1 within 8 s | Seen | Not seen — firmware crashed at boot or COM4 not connected |
| `PG: OK` in [SYSTEM] | Present | `PG: FAIL` — USB PD negotiation failed; TMC2209 disabled |
| `VBus` | Matches requested voltage (e.g., 20V ± 0.5V) | Out of range — check USB PD adapter |
| `Setup complete` | Optional — often missed due to fast reboot timing | — |

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `FileNotFoundError: COM3` | Wrong port or board not connected | Check Device Manager; ensure USB CDC driver installed |
| Upload hangs at "Connecting..." | Board not entering bootloader | Try pressing BOOT + RST on the board, then release RST |
| `Brownout` reset reason on Serial1 | USB PD voltage too low at boot | Check PD adapter rating; ensure `set_voltage` saved to flash |
| `Setup complete` missing | Inconclusive on its own — often missed due to reboot timing | Use `[SYSTEM]` presence as primary indicator; if `[SYSTEM]` also missing, firmware crashed |
| Flash % increases a lot after edits | Heap alloc in ISR or control task | Audit new code for `new`, `malloc`, `String` in real-time paths |
