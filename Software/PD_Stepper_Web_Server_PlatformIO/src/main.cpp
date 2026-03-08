#include "encoder.h"
#include "index_html.h"
#include "motion_control.h"
#include "tmc_driver.h"
#include "usb_telemetry_provider.h"
#include "web_server.h"
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

// Global variables
int set_speed = 0;
bool PGState = 0; // state of the power good signal from PD sink IC
bool enabledState = 0;
bool state = 0; // step state

// AS5600 Hall Effect Encoder
// (Logic moved to encoder.cpp)
unsigned long lastEncRead = 0;

int mainFreq =
    10; // Scheduled frequency = 100hz (for slower tasks, encoder reading etc)

// button read and debounce
bool incButtonState = HIGH;
bool decButtonState = HIGH;
bool resetButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int buttonSpeed = 0;

// Voltage reading and calc
float VBusVoltage = 0;
float VREF = 3.3;
const float DIV_RATIO = 0.1189427313; // 20k&2.7K Voltage Divider

// Values received from websever save command
String enabled1 = "enabled";
String setVoltage = "12";
String microsteps = "32";
String current = "30";
String stallThreshold = "10";
String standstillMode = "NORMAL";

// variable updated in callback
volatile bool speedUpdatePending = false;
volatile int pendingSpeed = 0;
volatile bool posUpdatePending = false;
volatile int pendingPosMode = 0;

// Varaiables for position control (open loop)
signed long setPoint = 0;
signed long CurrentPosition = 0;
unsigned long lastStep = 0;

// Note: Helper functions (readPGState, readVoltage, etc.) have been moved to
// web_server.cpp

// updates placeholder varibles in the HTML code (used by web_server.cpp)
String processor_REMOVED_SEE_WEB_SERVER_CPP(const String &var) {
  if (var == "enabled1") {
    if (enabled1 == "enabled") {
      return "checked";
    } else {
      return "";
    }
  }

  if (var == "microsteps") {
    return String(microsteps);
  }

  if (var == "voltage") {
    return String(setVoltage);
  }

  if (var == "current") {
    return String(current);
  }

  if (var == "stall_threshold") {
    return String(stallThreshold);
  }

  if (var == "standstill_mode") {
    return String(standstillMode);
  }
  return String("");
}

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
  encoder::startTask(5, 10); // 100Hz at Priority 5

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

  // Set up USB Serial for monitoring with pio
  USBSerial.begin(921600); // Must match StepperClientUSB.py --baud
  delay(500);              // Short stabilization time
  USBSerial.println("\r\n[SERIAL] Ready");
  USBSerial.flush();

  // Reset reason and boot counter
  esp_reset_reason_t reason = esp_reset_reason();
  preferences.begin("system", false);
  uint32_t bootCount = preferences.getUInt("boot_count", 0);
  bootCount++;
  preferences.putUInt("boot_count", bootCount);
  preferences.end();

  USBSerial.printf("\r\n--- SYSTEM BOOT #%u ---\r\n", bootCount);
  USBSerial.print("Reset Reason: ");
  switch (reason) {
  case ESP_RST_POWERON:
    USBSerial.println("Power-on");
    break;
  case ESP_RST_EXT:
    USBSerial.println("External Pin");
    break;
  case ESP_RST_SW:
    USBSerial.println("Software Reset");
    break;
  case ESP_RST_PANIC:
    USBSerial.println("Software Panic");
    break;
  case ESP_RST_INT_WDT:
    USBSerial.println("Interrupt Watchdog");
    break;
  case ESP_RST_TASK_WDT:
    USBSerial.println("Task Watchdog");
    break;
  case ESP_RST_WDT:
    USBSerial.println("Other Watchdog");
    break;
  case ESP_RST_DEEPSLEEP:
    USBSerial.println("Deep Sleep");
    break;
  case ESP_RST_BROWNOUT:
    USBSerial.println("Brownout");
    break;
  case ESP_RST_SDIO:
    USBSerial.println("SDIO Reset");
    break;
  default:
    USBSerial.println("Unknown");
    break;
  }

  // Initialize and start web server on core 0 to leave Core 1 for motion
  webserver::initWebServer(0);
  webserver::beginWebServer();

  digitalWrite(LED1, HIGH);
  delay(200);
  digitalWrite(LED1, LOW);
  // Initialize motion control system
  // Inject telemetry transport — swap to &wifiTelemetry to switch providers.
  motion::setTelemetryProvider(&usbTelemetry);
  motion::init();

  USBSerial.println("Setup complete");
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
          if (doc["cmd"].is<const char *>() && doc["cmd"] == "move") {
            long dist = doc["distance"] | 0;
            float accel = doc["accel"] | 1000.0f;
            float speed = doc["speed"] | 5000.0f;
            bool isAbs = doc["abs"] | false;
            USBSerial.printf(
                "%s Command - Target/Dist: %ld, Accel: %.2f, Speed: %.2f\n",
                isAbs ? "Absolute" : "Relative", dist, accel, speed);
            motion::addCommand(dist, accel, speed, isAbs);
          } else if (doc["cmd"].is<const char *>() &&
                     doc["cmd"] == "telemetry") {
            bool enabled = doc["enabled"] | false;
            USBSerial.printf("Telemetry Command: %s\n", enabled ? "ON" : "OFF");
          } else if (doc["cmd"].is<const char *>() && doc["cmd"] == "set_pid") {
            float kp = doc["kp"] | 3.0f;
            float ki = doc["ki"] | 0.05f;
            motion::setPID(kp, ki);
            USBSerial.printf("Set PID - Kp: %.4f, Ki: %.4f\n", kp, ki);
          }
        } else {
          USBSerial.printf("JSON Deserialization failed: %s\n", error.c_str());
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

    // Diagnostic output
    USBSerial.printf("[SYSTEM] VBus: %.2fV, PG: %s, Core: %d\r\n", VBusVoltage,
                     PGState ? "FAIL" : "OK", xPortGetCoreID());
    USBSerial.flush();
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
    USBSerial.println("Settings found in EEPROM");
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
  USBSerial.println("Saving settings to flash");
  preferences.end();
  configureSettings();
}
