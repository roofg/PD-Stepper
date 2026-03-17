---
name: read-aux-debug-serial
description: "Guide for reading and interpreting the PD-Stepper AUX debug serial output (Serial1 on COM4 via FT232RL). Use this when asked to open the debug monitor, interpret boot messages, diagnose motion errors, or understand DBG: output."
---

The PD-Stepper board outputs all debug text on Serial1 (AUX UART), never on USBSerial.
USBSerial is binary-only (telemetry packets). Do not mix them.

## Hardware

- AUX1 (GPIO 14, ESP32 TX) → FT232RL RXD
- GND → FT232RL GND
- FT232RL TXD → leave unconnected (5V would damage ESP32 GPIO)
- The FT232RL appears as COM4 on this machine (VID 0403 / PID 6001)

## Open the monitor (interactive)

```
platformio device monitor --port COM4 --baud 115200
```

Power-cycle the board after opening — boot messages fire before the terminal connects.

## Read programmatically (Python)

Use this when you need to capture output non-interactively (e.g., to extract a specific value or wait for a specific line). `pyserial` is available in the project venv (`requirements.txt`).

```python
import serial, time

PORT = "COM4"
BAUD = 115200
TIMEOUT_SEC = 5  # how long to wait for the target line

s = serial.Serial(PORT, BAUD, timeout=3, dsrdtr=True)
time.sleep(0.2)  # let port settle

deadline = time.time() + TIMEOUT_SEC
lines = []
while time.time() < deadline:
    line = s.readline().decode("utf-8", errors="replace").strip()
    if line:
        lines.append(line)
        if "[SYSTEM]" in line:  # change target pattern as needed
            break
s.close()

for l in lines:
    print(l)
```

Key parameters:
- `dsrdtr=True` — prevents ESP32 reset on port open/close (critical)
- `timeout=3` — readline will return after 3 s even if no newline arrives
- Change `"[SYSTEM]"` to any other prefix (`"DBG:"`, `"ERR:"`, etc.) to wait for a different line type
- Increase `TIMEOUT_SEC` if waiting for an event that takes longer (e.g., end of a move)

## Boot sequence

```
--- SYSTEM BOOT #N ---
Reset Reason: Power-on
[SERIAL] Ready
Setup complete
```

`Reset Reason` values: Power-on, External Pin, Software Reset, Software Panic, Interrupt Watchdog, Task Watchdog, Brownout. Repeated boots with incrementing #N = crash loop.

## 1 Hz system diagnostics

```
[SYSTEM] VBus: 20.05V, PG: OK, Core: 1
```

- `VBus` — measured supply voltage; should match requested PD voltage
- `PG: OK` — USB PD Power Good; `PG: FAIL` = PD negotiation failed, TMC2209 stays disabled

## Command echo (on JSON command receipt)

```
Set PD - Kp: 3.0000, Kd: 0.1000
Set phase lead gain Kv: 0.0020
Set voltage: 20 V
Set current: 80%
Set microsteps: 32
Set stall threshold: 10
Set standstill mode: NORMAL
Settings saved to flash
ERR: motion queue full, command dropped
JSON Deserialization failed: <reason>
Unknown command: <cmd>
```

## Motion/planner debug (DBG: prefix)

```
DBG:PLANNER 1 cmd(s) queued
DBG:PLAN[0] dist=3200 fwd=1 entry=0 cruise=12000 exit=0
DBG:BLOCK[0] start=0 end=3200 entry=0 exit=0
DBG:PLANNER_DONE reason=NORMAL
DBG:TMC_DISABLE_START
DBG:TMC_DISABLE_DONE
DBG:STOP_SENDING pos=3200
DBG:STOP_SENT
```

`reason` values on PLANNER_DONE: `NORMAL` (completed cleanly), `ESTOP` (following error exceeded threshold — motor stalled or encoder disconnected).

## Troubleshooting

| Symptom | Cause |
|---|---|
| No output at all | Wrong port, wrong baud (must be 115200), or AUX1/GND miswired |
| `PG: FAIL` in [SYSTEM] | USB PD supply not providing requested voltage |
| `ERR: motion queue full` | Commands sent faster than motion completes |
| `JSON Deserialization failed` | Malformed JSON on USBSerial; check newline termination |
| `PLANNER_DONE reason=ESTOP` | Following error exceeded threshold; check encoder connection |
| Boot # incrementing rapidly | Crash loop; check Reset Reason for watchdog/panic |
