#include "encoder.h"
#include "motion_control.h"
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

// TMC2209 pins/config (kept here; driver uses these constants)
#define TMC_EN 21
#define STEP 5
#define DIR 6
#define MS1 1
#define MS2 2
#define SPREAD 7
#define TMC_TX 17
#define TMC_RX 18
#define DIAG 16
#define INDEX 11

// PD Trigger (CH224K)
#define PG 15 // power good singnal (dont enable stepper untill this is good)
#define CFG1 38
#define CFG2 48
#define CFG3 47

// Other
#define VBUS 4
#define NTC 7
#define LED1 10
#define LED2 12
#define SW1 35
#define SW2 36
#define SW3 37
#define AUX1 14
#define AUX2 13

// Runtime state
bool PGState = 0;
float VBusVoltage = 0;
const float DIV_RATIO = 0.1189427313; // 20k/2.7k voltage divider

// Persistent settings (loaded from Preferences on boot, saved by "save" command)
String enabled1       = "enabled";
String setVoltage     = "20";
String microsteps     = "32";
String current        = "50";
String stallThreshold = "10";
String standstillMode = "NORMAL";

// Note: button debounce and open-loop position control variables have been
// removed — the new motion architecture handles all motion via serial commands.

// Arduino framwork setup defaultly runs on core 1
void setup() {
  esp_task_wdt_delete(NULL); // Stop monitoring loopTask
  // PD Trigger Setup
  pinMode(PG, INPUT);
  pinMode(CFG1, OUTPUT);
  pinMode(CFG2, OUTPUT);
  pinMode(CFG3, OUTPUT);
  digitalWrite(CFG1, LOW);
  digitalWrite(CFG2, LOW);
  digitalWrite(CFG3, HIGH);

  // General
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);

  // TMC pins
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(TMC_EN, OUTPUT);
  pinMode(DIAG, INPUT);
  digitalWrite(TMC_EN, LOW);
  digitalWrite(MS2, LOW);

  // AS5600 Hall Encoder Setup
  encoder::init();
  encoder::startTask(5, 5); // 200 Hz (5 ms interval), priority 5

  // ADC Setup
  analogSetPinAttenuation(VBUS, ADC_11db);

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
  // AUX1 (GPIO 14) = TX from ESP32; AUX2 (GPIO 13) = RX into ESP32.
  // Connect a USB-UART adapter to AUX1+GND to read debug output.
  Serial1.begin(115200, SERIAL_8N1, AUX2, AUX1);

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

  digitalWrite(LED1, HIGH);
  delay(200);
  digitalWrite(LED1, LOW);
  // Initialize motion control system
  // Inject telemetry transport — swap to &wifiTelemetry to switch providers.
  motion::setTelemetryProvider(&usbTelemetry);
  motion::init();
  motion::setMicrosteps(microsteps.toInt());

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
            const char *v = doc["value"] | "20";
            setVoltage = String(v);
            configureSettings();
            Serial1.printf("Set voltage: %s V\n", v);

          } else if (strcmp(cmd, "set_current") == 0) {
            int c = doc["value"] | 50;
            current = String(c);
            tmc::setRunCurrent(c);
            Serial1.printf("Set current: %d%%\n", c);

          } else if (strcmp(cmd, "set_microsteps") == 0) {
            int ms = doc["value"] | 32;
            microsteps = String(ms);
            tmc::setMicrostepsPerStep(ms);
            motion::setMicrosteps(ms);
            Serial1.printf("Set microsteps: %d\n", ms);

          } else if (strcmp(cmd, "set_stall_threshold") == 0) {
            int th = doc["value"] | 10;
            stallThreshold = String(th);
            tmc::setStallGuardThreshold(th);
            Serial1.printf("Set stall threshold: %d\n", th);

          } else if (strcmp(cmd, "set_standstill_mode") == 0) {
            const char *mode = doc["value"] | "NORMAL";
            standstillMode = String(mode);
            configureSettings();
            Serial1.printf("Set standstill mode: %s\n", mode);

          } else if (strcmp(cmd, "save") == 0) {
            writeSettings();
            Serial1.println("Settings saved to flash");

          } else if (strcmp(cmd, "get_settings") == 0) {
            Serial1.printf(
                "{\"voltage\":\"%s\",\"current\":\"%s\",\"microsteps\":\"%s\","
                "\"stall_threshold\":\"%s\",\"standstill_mode\":\"%s\"}\n",
                setVoltage.c_str(), current.c_str(), microsteps.c_str(),
                stallThreshold.c_str(), standstillMode.c_str());

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
  processSerialCommands();

  static uint32_t lastPrintTime = 0;
  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    // VBus sampling (safe to do at 1Hz on Core 1)
    float vbus_mv = (float)analogReadMilliVolts(4);    // VBUS pin
    VBusVoltage = (vbus_mv / 1000.0f) / 0.1189427313f; // DIV_RATIO
    PGState = digitalRead(15);                         // PG pin

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
  if (setVoltage == "5") {
    digitalWrite(CFG1, HIGH);
  } else if (setVoltage == "9") {
    digitalWrite(CFG1, LOW);
    digitalWrite(CFG2, LOW);
    digitalWrite(CFG3, LOW);
  } else if (setVoltage == "12") {
    digitalWrite(CFG1, LOW);
    digitalWrite(CFG2, LOW);
    digitalWrite(CFG3, HIGH);
  } else if (setVoltage == "15") {
    digitalWrite(CFG1, LOW);
    digitalWrite(CFG2, HIGH);
    digitalWrite(CFG3, HIGH);
  } else if (setVoltage == "20") {
    digitalWrite(CFG1, LOW);
    digitalWrite(CFG2, HIGH);
    digitalWrite(CFG3, LOW);
  }

  tmc::setRunCurrent(current.toInt());
  tmc::setMicrostepsPerStep(microsteps.toInt());
  tmc::setStallGuardThreshold(stallThreshold.toInt());

  if (standstillMode == "NORMAL") {
    tmc::setStandstillMode(0);
  } // map modes in driver
  else if (standstillMode == "FREEWHEELING") {
    tmc::setStandstillMode(1);
  } else if (standstillMode == "BRAKING") {
    tmc::setStandstillMode(2);
  } else if (standstillMode == "STRONG_BRAKING") {
    tmc::setStandstillMode(3);
  }
}

void readSettings() {
  preferences.begin("settings", false);
  enabled1 = preferences.getString("enable", "");
  if (enabled1 == "") {
    preferences.end();
    enabled1 = "enabled";
    setVoltage = "20";
    microsteps = "32";
    current = "50";
    stallThreshold = "10";
    standstillMode = "NORMAL";
    writeSettings();
  } else {
    Serial1.println("Settings found in EEPROM");
    setVoltage = preferences.getString("voltage", "");
    if (setVoltage == "12") {
      setVoltage = "20"; // Migration to higher PD voltage for test
      preferences.putString("voltage", "20");
    }
    microsteps = preferences.getString("microsteps", "");
    current = preferences.getString("current", "");
    if (current == "80" || current == "30") {
      current = "50"; // Safer middle ground for high speed
      preferences.putString("current", "50");
    }
    stallThreshold = preferences.getString("stallThreshold", "");
    standstillMode = preferences.getString("standstillMode", "");
    preferences.end();
  }
}

void writeSettings() {
  preferences.begin("settings", false);
  preferences.putString("enable", enabled1);
  preferences.putString("voltage", setVoltage);
  preferences.putString("microsteps", microsteps);
  preferences.putString("current", current);
  preferences.putString("stallThreshold", stallThreshold);
  preferences.putString("standstillMode", standstillMode);
  Serial1.println("Saving settings to flash");
  preferences.end();
  configureSettings();
}
