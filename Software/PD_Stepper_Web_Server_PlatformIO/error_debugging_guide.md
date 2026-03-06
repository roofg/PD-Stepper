# PD-Stepper Connection & Movement Debugging Guide

This guide explains common errors you might encounter when running `StepperClientPython.py` to command the ESP32 motor controller.

## 1. Connection Errors (Python Client)

### `TimeoutError` or `Connection timed out`
*   **What it means:** The script cannot find the ESP32 on the network.
*   **Likely Cause:** 
    *   You are not connected to the **"PD Stepper"** Wi-Fi Access Point.
    *   The ESP32 is powered off.
*   **Solution:** Check your Wi-Fi settings and ensure you are connected to the ESP32's AP.

### `ConnectionRefusedError`
*   **What it means:** The ESP32 is found, but it rejected the connection.
*   **Likely Cause:** 
    *   The Web Server crashed on the ESP32.
    *   Another client is already taking up the single available WebSocket slot (if limited by firmware).
*   **Solution:** Reboot the ESP32.

### `Connection closed unexpectedly`
*   **What it means:** The connection was established but then dropped.
*   **Likely Cause:** 
    *   ESP32 rebooted (possibly due to a power surge or software crash).
    *   The Wi-Fi signal is weak or unstable.
*   **Solution:** Check the ESP32 Serial Monitor to see if it rebooted. Ensure the motor power supply is stable.

---

## 2. Movement Errors (ESP32 Firmware)

These are reported via the `"stop"` message from the firmware.

### `Lag Fault`
*   **What it means:** The difference between the **Target Position** and **Measured Position** (Encoder) exceeded the allowed threshold (usually 200+ steps).
*   **Likely Causes:**
    1.  **Mechanical Obstruction:** The motor is physically blocked.
    2.  **Too Fast:** The commanded `speed` or `accel` is higher than the motor can handle for the given load.
    3.  **Insufficient Current:** The TMC driver `current` setting is too low.
    4.  **Wiring Issue:** The motor phases or encoder cable might have a loose connection.
*   **Solution:** Reduce `accel` and `speed` in your Python script. Increase the current in the Web UI if necessary.

### `E-STOP (SW1)`
*   **What it means:** The physical button SW1 on the PCB was pressed.
*   **Likely Cause:** User interaction or a hardware short on the SW1 pin.
*   **Solution:** Check if the button is stuck.

---

## 3. Data Format Errors

### `Received malformed JSON`
*   **What it means:** The data received from the ESP32 wasn't valid JSON.
*   **Likely Cause:** Memory corruption or partial packet transmission.
*   **Solution:** Ensure you are using the latest firmware and that the ESP32 isn't running out of heap memory.
