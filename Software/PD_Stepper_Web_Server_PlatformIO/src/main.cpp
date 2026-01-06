#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

#include "dual_core_scaffold.h"
#include "encoder.h"
#include "index_html.h"
#include "tmc_driver.h"
#include "web_server.h"


Preferences preferences;

// forward declarations (needed because we're compiling as C++ source)
void configureSettings();
void readSettings();
void writeSettings();

// access point SSID and password (password = "" for no password)
const char *ssid = "PD Stepper";
const char *password = "";

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
  pinMode(MS1, OUTPUT);
  pinMode(TMC_EN, OUTPUT);
  pinMode(DIAG, INPUT);
  digitalWrite(TMC_EN, LOW);
  digitalWrite(MS2, LOW);

  // AS5600 Hall Encoder Setup
  encoder::init();

  // ADC Setup
  analogSetPinAttenuation(VBUS, ADC_11db);

  readSettings(); // get saved values from EEPROM

  tmc::init(TMC_RX, TMC_TX);
  tmc::setRunCurrent(20);
  tmc::enableAutomaticCurrentScaling();
  tmc::enableStealthChop();
  tmc::setCoolStepDurationThreshold(5000);
  tmc::disable();

  configureSettings(); // use saved settings

  // Set up USB Serial for monitoring with pio
  USBSerial.begin(115200); // Baud rate often ignored for native USB
  delay(2000);             // Give some time for the USB serial to initialize
  USBSerial.println("SerialUSB ready!");
  USBSerial.flush();

  // Initialize and start web server on core 1
  webserver::initWebServer(1);
  webserver::beginWebServer();

  digitalWrite(LED1, HIGH);
  delay(200);
  digitalWrite(LED1, LOW);
  // To enable the dual-core demo (controller pinned to core 0, network on core
  // 1), uncomment the next line. The demo creates example tasks and a queue.
  start_dual_core_demo();

  //   delay(2000);

  //   stop_dual_core_demo();
  USBSerial.println("Setup complete");
}

// Arduino framwork main loop defaultly runs on core 1
void loop() {

  digitalWrite(LED1, HIGH);
  delay(1000);
  digitalWrite(LED1, LOW);
  delay(1000);
  USBSerial.printf("Main loop running on core %d\n", xPortGetCoreID());

  //   if (speedUpdatePending) {
  //     set_speed = pendingSpeed;
  //     tmc::moveAtVelocity(set_speed * (microsteps.toInt()));
  //     speedUpdatePending = false;
  //   }

  //   if (posUpdatePending) {
  //     tmc::moveAtVelocity(0);
  //     if (pendingPosMode == 1)      setPoint -= 25600;
  //     else if (pendingPosMode == 2) setPoint -= 12800;
  //     else if (pendingPosMode == 3) setPoint += 12800;
  //     else if (pendingPosMode == 4) setPoint += 25600;
  //     posUpdatePending = false;
  //   }

  //   if (millis() - lastEncRead >= mainFreq){
  //     lastEncRead = millis();
  //     digitalWrite(LED2, digitalRead(DIAG));
  //     PGState = digitalRead(PG);
  //     if (PGState == LOW and enabled1 == "enabled" and enabledState == 0){
  //       tmc::enable();
  //       enabledState = 1;
  //     } else if ((PGState == HIGH or enabled1 == "disabled") and enabledState
  //     == 1){
  //       tmc::disable();
  //       enabledState = 0;
  //     }
  //   }

  //   int delaySpeed = 4500;
  //   int microSteps = microsteps.toInt();
  //   int delaySpeedAdjusted = delaySpeed/microSteps;
  //   if (setPoint > CurrentPosition){
  //     if (micros()-lastStep > delaySpeedAdjusted){
  //       digitalWrite(DIR, LOW);
  //       digitalWrite(STEP, state);
  //       state = !state;
  //       CurrentPosition = CurrentPosition + (256/microSteps);
  //       lastStep = micros();
  //     }
  //   } else if (setPoint < CurrentPosition){
  //     if (micros()-lastStep > delaySpeedAdjusted){
  //       digitalWrite(DIR, HIGH);
  //       digitalWrite(STEP, state);
  //       state = !state;
  //       CurrentPosition = CurrentPosition - (256/microSteps);
  //       lastStep = micros();
  //     }
  //   }

  //   if ((millis() - lastDebounceTime) > debounceDelay) {
  //     lastDebounceTime = millis();
  //     bool currentIncButtonState = digitalRead(SW3);
  //     bool currentDecButtonState = digitalRead(SW1);
  //     bool currentResetButtonState = digitalRead(SW2);

  //     if (currentIncButtonState != incButtonState) {
  //       incButtonState = currentIncButtonState;
  //       if (incButtonState == LOW) {
  //         buttonSpeed = buttonSpeed + 30;
  //         if (buttonSpeed > 330){
  //           buttonSpeed = 330;
  //         }
  //         tmc::moveAtVelocity(buttonSpeed*(microsteps.toInt()));
  //       }
  //     }

  //     if (currentDecButtonState != decButtonState) {
  //       decButtonState = currentDecButtonState;
  //       if (decButtonState == LOW) {
  //         buttonSpeed = buttonSpeed -30;
  //         if (buttonSpeed < -330){
  //           buttonSpeed = -330;
  //         }
  //         tmc::moveAtVelocity(buttonSpeed*(microsteps.toInt()));
  //       }
  //     }

  //     if (currentResetButtonState != resetButtonState) {
  //       resetButtonState = currentResetButtonState;
  //       if (resetButtonState == LOW) {
  //         buttonSpeed = 0;
  //         tmc::moveAtVelocity(0);
  //       }
  //     }
  //   }
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
    setVoltage = "12";
    microsteps = "32";
    current = "30";
    stallThreshold = "10";
    standstillMode = "NORMAL";
    writeSettings();
  } else {
    USBSerial.println("Settings found in EEPROM");
    setVoltage = preferences.getString("voltage", "");
    microsteps = preferences.getString("microsteps", "");
    current = preferences.getString("current", "");
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
