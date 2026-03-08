#include "encoder.h"
#include "motion_control.h"
#include "pins.h"
#include "tmc_driver.h"
#include "usb_telemetry_provider.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

Preferences preferences;

// Telemetry provider (USB binary — swappable via TelemetryProvider interface)
static UsbTelemetryProvider usbTelemetry;

// forward declarations (needed because we're compiling as C++ source)
void configureSettings();
void readSettings();
void writeSettings();

// Runtime state
bool PGState = 0;
float VBusVoltage = 0;
const float DIV_RATIO = 0.1189427313; // 20k/2.7k voltage divider

// Persistent settings (loaded from Preferences on boot, saved by "save" command)
static int   setVoltage     = 20;
static int   setMicrosteps  = 32;
static int   setCurrent     = 50;
static int   setStall       = 10;
static char  standstillMode[16] = "NORMAL";

// Note: button debounce and open-loop position control variables have been
// removed — the new motion architecture handles all motion via serial commands.

// Arduino framwork setup defaultly runs on core 1
void setup() {
  esp_task_wdt_reset(); // Feed WDT — loopTask is registered by Arduino framework
  // PD Trigger Setup
  pinMode(PD_PG, INPUT);
  pinMode(PD_CFG1, OUTPUT);
  pinMode(PD_CFG2, OUTPUT);
  pinMode(PD_CFG3, OUTPUT);
  digitalWrite(PD_CFG1, LOW);
  digitalWrite(PD_CFG2, LOW);
  digitalWrite(PD_CFG3, HIGH);

  // General
  pinMode(PIN_SW1, INPUT);
  pinMode(PIN_SW2, INPUT);
  pinMode(PIN_SW3, INPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(TMC_STEP, OUTPUT);
  pinMode(TMC_DIR, OUTPUT);

  // TMC pins
  pinMode(TMC_MS1, OUTPUT);
  pinMode(TMC_MS2, OUTPUT);
  pinMode(TMC_EN, OUTPUT);
  pinMode(TMC_DIAG, INPUT);
  digitalWrite(TMC_EN, LOW);
  digitalWrite(TMC_MS2, LOW);

  // AS5600 Hall Encoder Setup
  encoder::init();
  encoder::startTask(5, 1); // 1 kHz (1 ms interval) — AS5600 I2C read ~40µs at 400kHz

  // ADC Setup
  analogSetPinAttenuation(PIN_VBUS, ADC_11db);

  readSettings(); // get saved values from EEPROM

  tmc::init(TMC_RX, TMC_TX);
  tmc::setRunCurrent(80); // 80% current is safer for 12kHz moves
  tmc::enableAutomaticCurrentScaling();
  tmc::enableStealthChop(); // StealthChop is smoother for low/mid speeds
  tmc::setCoolStepDurationThreshold(5000);
  tmc::disable();

  configureSettings(); // use saved settings

  // AUX UART — all human-readable debug output goes here (Serial1), keeping
  // USBSerial strictly for binary telemetry packets + JSON command input.
  // PIN_AUX1 (GPIO 14) = TX from ESP32; PIN_AUX2 (GPIO 13) = RX into ESP32.
  // Connect a USB-UART adapter to PIN_AUX1+GND to read debug output.
  Serial1.begin(115200, SERIAL_8N1, PIN_AUX2, PIN_AUX1);

  // USB CDC — binary protocol only (no text written to USBSerial anywhere).
  USBSerial.begin(921600); // Must match TriggerMove.py --baud
  delay(500);              // Short stabilization time
  Serial1.println("\r\n[SERIAL] Ready");

  // Reset reason and boot counter
  esp_reset_reason_t reason = esp_reset_reason();
  preferences.begin("system", false);
  uint32_t bootCount = preferences.getUInt("boot_count", 0);
  bootCount++;
  preferences.putUInt("boot_count", bootCount);
  preferences.end();

  Serial1.printf("\r\n--- SYSTEM BOOT #%u ---\r\n", bootCount);
  Serial1.print("Reset Reason: ");
  switch (reason) {
  case ESP_RST_POWERON:
    Serial1.println("Power-on");
    break;
  case ESP_RST_EXT:
    Serial1.println("External Pin");
    break;
  case ESP_RST_SW:
    Serial1.println("Software Reset");
    break;
  case ESP_RST_PANIC:
    Serial1.println("Software Panic");
    break;
  case ESP_RST_INT_WDT:
    Serial1.println("Interrupt Watchdog");
    break;
  case ESP_RST_TASK_WDT:
    Serial1.println("Task Watchdog");
    break;
  case ESP_RST_WDT:
    Serial1.println("Other Watchdog");
    break;
  case ESP_RST_DEEPSLEEP:
    Serial1.println("Deep Sleep");
    break;
  case ESP_RST_BROWNOUT:
    Serial1.println("Brownout");
    break;
  case ESP_RST_SDIO:
    Serial1.println("SDIO Reset");
    break;
  default:
    Serial1.println("Unknown");
    break;
  }

  // Initialize and start web server on core 0 to leave Core 1 for motion
  // Web server removed — all commanding is via USB Serial JSON.

  digitalWrite(PIN_LED1, HIGH);
  delay(200);
  digitalWrite(PIN_LED1, LOW);
  // Initialize motion control system
  // Inject telemetry transport — swap to &wifiTelemetry to switch providers.
  motion::setTelemetryProvider(&usbTelemetry);
  motion::init();
  motion::setMicrosteps(setMicrosteps);
  motion::setConfiguredVoltage((float)setVoltage); // derive brownout threshold

  Serial1.println("Setup complete");
}

void processSerialCommands() {
  static char serialBuffer[512];
  static int bufIndex = 0;

  while (USBSerial.available() > 0) {
    char c = USBSerial.read();
    if (c == '\n' || c == '\r') {
      if (bufIndex > 0) {
        serialBuffer[bufIndex] = '\0';
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, serialBuffer);

        if (!error) {
          const char *cmd = doc["cmd"] | "";

          if (strcmp(cmd, "move") == 0) {
            long dist = doc["distance"] | 0;
            float accel = doc["accel"] | 1000.0f;
            float speed = doc["speed"] | 5000.0f;
            bool isAbs = doc["abs"] | false;
            Serial1.printf(
                "%s Command - Target/Dist: %ld, Accel: %.2f, Speed: %.2f\n",
                isAbs ? "Absolute" : "Relative", dist, accel, speed);
            motion::addCommand(dist, accel, speed, isAbs);

          } else if (strcmp(cmd, "set_phase_lead") == 0) {
            float kv = doc["kv"] | 0.0f;
            motion::setPhaseLeadGain(kv);
            Serial1.printf("Set phase lead gain Kv: %.4f\n", kv);

          } else if (strcmp(cmd, "set_pd") == 0) {
            float kp = doc["kp"] | 3.0f;
            float kd = doc["kd"] | 0.1f;
            motion::setPD(kp, kd);
            Serial1.printf("Set PD - Kp: %.4f, Kd: %.4f\n", kp, kd);

          } else if (strcmp(cmd, "set_pid") == 0) {
            // Legacy alias: map ki → kd for backwards compat with scripts
            float kp = doc["kp"] | 3.0f;
            float kd = doc["kd"] | doc["ki"] | 0.1f;
            motion::setPD(kp, kd);
            Serial1.printf("Set PD (legacy set_pid) - Kp: %.4f, Kd: %.4f\n", kp, kd);

          } else if (strcmp(cmd, "set_voltage") == 0) {
            setVoltage = doc["value"] | 20;
            configureSettings();
            motion::setConfiguredVoltage((float)setVoltage);
            Serial1.printf("Set voltage: %d V\n", setVoltage);

          } else if (strcmp(cmd, "set_current") == 0) {
            setCurrent = doc["value"] | 50;
            tmc::setRunCurrent(setCurrent);
            Serial1.printf("Set current: %d%%\n", setCurrent);

          } else if (strcmp(cmd, "set_microsteps") == 0) {
            setMicrosteps = doc["value"] | 32;
            tmc::setMicrostepsPerStep(setMicrosteps);
            motion::setMicrosteps(setMicrosteps);
            Serial1.printf("Set microsteps: %d\n", setMicrosteps);

          } else if (strcmp(cmd, "set_stall_threshold") == 0) {
            setStall = doc["value"] | 10;
            tmc::setStallGuardThreshold(setStall);
            Serial1.printf("Set stall threshold: %d\n", setStall);

          } else if (strcmp(cmd, "set_standstill_mode") == 0) {
            const char *mode = doc["value"] | "NORMAL";
            strncpy(standstillMode, mode, sizeof(standstillMode) - 1);
            standstillMode[sizeof(standstillMode) - 1] = '\0';
            configureSettings();
            Serial1.printf("Set standstill mode: %s\n", standstillMode);

          } else if (strcmp(cmd, "save") == 0) {
            writeSettings();
            Serial1.println("Settings saved to flash");

          } else if (strcmp(cmd, "get_settings") == 0) {
            Serial1.printf(
                "{\"voltage\":%d,\"current\":%d,\"microsteps\":%d,"
                "\"stall_threshold\":%d,\"standstill_mode\":\"%s\"}\n",
                setVoltage, setCurrent, setMicrosteps,
                setStall, standstillMode);

          } else if (strcmp(cmd, "telemetry") == 0) {
            bool enabled = doc["enabled"] | false;
            Serial1.printf("Telemetry Command: %s\n", enabled ? "ON" : "OFF");

          } else {
            Serial1.printf("Unknown command: %s\n", cmd);
          }
        } else {
          Serial1.printf("JSON Deserialization failed: %s\n", error.c_str());
        }
        bufIndex = 0;
      }
    } else {
      if (bufIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[bufIndex++] = c;
      }
    }
  }
}

void loop() {
  esp_task_wdt_reset(); // Feed watchdog — loopTask is monitored by the Arduino WDT
  processSerialCommands();

  static uint32_t lastPrintTime = 0;
  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    // VBus sampling (safe to do at 1Hz on Core 1)
    float vbus_mv = (float)analogReadMilliVolts(PIN_VBUS);
    VBusVoltage = (vbus_mv / 1000.0f) / 0.1189427313f; // DIV_RATIO
    PGState = digitalRead(PD_PG);

    // Diagnostic output to AUX UART (Serial1), not USB CDC
    Serial1.printf("[SYSTEM] VBus: %.2fV, PG: %s, Core: %d\r\n", VBusVoltage,
                     PGState ? "FAIL" : "OK", xPortGetCoreID());
  }

  // Explicitly yield to reset the loopTask watchdog
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

// Encoder logic moved to encoder::read() in encoder.cpp

/// @brief Setting pin combination negotiates USB-PD voltage. This voltage is
/// passed to the TMC driver as motor supply voltage.
void configureSettings() {
  // CH224K CFG pins select USB-PD negotiated voltage
  if (setVoltage == 5) {
    digitalWrite(PD_CFG1, HIGH);
    digitalWrite(PD_CFG2, LOW);
    digitalWrite(PD_CFG3, LOW);
  } else if (setVoltage == 9) {
    digitalWrite(PD_CFG1, LOW);
    digitalWrite(PD_CFG2, LOW);
    digitalWrite(PD_CFG3, LOW);
  } else if (setVoltage == 12) {
    digitalWrite(PD_CFG1, LOW);
    digitalWrite(PD_CFG2, LOW);
    digitalWrite(PD_CFG3, HIGH);
  } else if (setVoltage == 15) {
    digitalWrite(PD_CFG1, LOW);
    digitalWrite(PD_CFG2, HIGH);
    digitalWrite(PD_CFG3, HIGH);
  } else if (setVoltage == 20) {
    digitalWrite(PD_CFG1, LOW);
    digitalWrite(PD_CFG2, HIGH);
    digitalWrite(PD_CFG3, LOW);
  }

  tmc::setRunCurrent(setCurrent);
  tmc::setMicrostepsPerStep(setMicrosteps);
  tmc::setStallGuardThreshold(setStall);

  if (strcmp(standstillMode, "FREEWHEELING") == 0) {
    tmc::setStandstillMode(1);
  } else if (strcmp(standstillMode, "BRAKING") == 0) {
    tmc::setStandstillMode(2);
  } else if (strcmp(standstillMode, "STRONG_BRAKING") == 0) {
    tmc::setStandstillMode(3);
  } else {
    tmc::setStandstillMode(0); // NORMAL (default)
  }
}

void readSettings() {
  preferences.begin("settings", false);
  bool hasSettings = preferences.isKey("voltage");
  if (!hasSettings) {
    preferences.end();
    // Defaults already assigned by initializers — persist them
    writeSettings();
    Serial1.println("No settings found — writing defaults");
  } else {
    Serial1.println("Settings found in EEPROM");
    setVoltage    = preferences.getInt("voltage",        20);
    setMicrosteps = preferences.getInt("microsteps",     32);
    setCurrent    = preferences.getInt("current",        50);
    setStall      = preferences.getInt("stallThreshold", 10);
    String mode   = preferences.getString("standstillMode", "NORMAL");
    strncpy(standstillMode, mode.c_str(), sizeof(standstillMode) - 1);
    standstillMode[sizeof(standstillMode) - 1] = '\0';
    preferences.end();
  }
}

void writeSettings() {
  preferences.begin("settings", false);
  preferences.putInt("voltage",        setVoltage);
  preferences.putInt("microsteps",     setMicrosteps);
  preferences.putInt("current",        setCurrent);
  preferences.putInt("stallThreshold", setStall);
  preferences.putString("standstillMode", standstillMode);
  Serial1.println("Saving settings to flash");
  preferences.end();
  configureSettings();
}
