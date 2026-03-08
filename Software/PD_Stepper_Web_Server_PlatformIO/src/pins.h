#pragma once

// PD-Stepper PCB — ESP32-S3 GPIO pin assignments
// Single source of truth. Include this header wherever pin numbers are needed.
// Do NOT re-define any of these constants in other translation units.

// TMC2209 stepper driver
#define TMC_EN   21
#define TMC_STEP  5
#define TMC_DIR   6
#define TMC_MS1   1
#define TMC_MS2   2
#define TMC_SPREAD 7
#define TMC_TX   17
#define TMC_RX   18
#define TMC_DIAG 16
#define TMC_INDEX 11

// CH224K USB-PD controller
#define PD_PG    15   // Power-Good — active HIGH; poll before enabling TMC
#define PD_CFG1  38
#define PD_CFG2  48
#define PD_CFG3  47

// Analog monitoring
#define PIN_VBUS  4   // VBus sense (voltage divider: 20k / 2.7k, ratio 0.1189)
#define PIN_NTC   7   // NTC thermistor

// LEDs (active HIGH)
#define PIN_LED1 10
#define PIN_LED2 12

// User buttons (active LOW, internal pull-up)
#define PIN_SW1  35
#define PIN_SW2  36
#define PIN_SW3  37

// AUX UART (debug output — Serial1)
// AUX1 = ESP32 TX (connect adapter RX here)
// AUX2 = ESP32 RX (leave unconnected for monitor-only use)
#define PIN_AUX1 14
#define PIN_AUX2 13
