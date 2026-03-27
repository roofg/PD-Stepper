#include "encoder.h"
#include "homing.h"
#include "motion_control.h"
#include "pins.h"
#include "step_generator.h"
#include "tmc_driver.h"
#include "usb_telemetry_provider.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

Preferences preferences;

// USB write mutex — serialises USBSerial.write() across TelemetryTask, DiagnosticsTask,
// and sendSettingsPacket() (declared in usb_telemetry_provider.h as extern).
SemaphoreHandle_t g_usbWriteMutex = nullptr;

// Telemetry provider (USB binary — swappable via TelemetryProvider interface)
static UsbTelemetryProvider usbTelemetry;

// forward declarations (needed because we're compiling as C++ source)
void configureSettings();
void readSettings();
void writeSettings();
void sendSettingsPacket();

// Runtime state — volatile for safe cross-core reads (loop() Core 1, DiagnosticsTask Core 0)
volatile bool  PGState     = 0;
volatile float VBusVoltage = 0;
const float DIV_RATIO = 0.1189427313; // 20k/2.7k voltage divider

// Boot counter — file-scope global so DiagnosticsTask can read it via extern
uint32_t bootCount = 0;

// Persistent settings (loaded from Preferences on boot, saved by "save" command)
static int   setVoltage     = 20;
static int   setMicrosteps  = 32;
static int   setCurrent     = 60;  // 60% default — was 80%; lower heat, still sufficient torque
static int   setStall       = 10;
static int   setHoldCurrent = 25;
static int   setHoldDelay   = 8;
static char  standstillMode[16] = "NORMAL";
static bool  stealthchopEnabled = true;
static bool  coolstepEnabled    = true;
static int   setSpreadCycleSpeed = 0;  // steps/s threshold for StealthChop→SpreadCycle (0 = disabled)
static float setKp     = 3.0f;
static float setKd     = 0.1f;
static float setKv     = 0.0f;
static float setDAlpha = 0.8f;
static float setJerk   = 0.0f;  // ramp time in ms; 0 = auto (~10 ms, trapezoidal)

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
  tmc::setRunCurrent(80); // 80% initial; configureSettings() will lower to setCurrent (60% default)
  tmc::setHoldCurrent(25); // 25% hold — low heat, maintains position
  tmc::enableAutomaticCurrentScaling();
  tmc::enableAutomaticGradientAdaptation(); // improves PWM efficiency alongside auto scaling
  tmc::enableStealthChop(); // StealthChop is smoother for low/mid speeds
  tmc::setCoolStepDurationThreshold(2000); // CoolStep active when TSTEP < 2000 (above min speed)
  tmc::enableCoolStep(1, 0);  // lower=1 (reduce 1 step), upper=0 (increase immediately)
  tmc::setPowerDownDelay(10); // fast IRUN→IHOLD transition after standstill detected
  tmc::disable();

  configureSettings(); // use saved settings

  // AUX UART — all human-readable debug output goes here (Serial1), keeping
  // USBSerial strictly for binary telemetry packets + JSON command input.
  // PIN_AUX1 (GPIO 14) = TX from ESP32; PIN_AUX2 (GPIO 13) = RX into ESP32.
  // Connect a USB-UART adapter to PIN_AUX1+GND to read debug output.
  Serial1.begin(115200, SERIAL_8N1, PIN_AUX2, PIN_AUX1);

  // USB CDC — binary protocol only (no text written to USBSerial anywhere).
  USBSerial.setRxBufferSize(1024); // Default 256 B is insufficient for burst of 9+1 settings+move commands (~420 B)
  USBSerial.begin(921600); // Must match TriggerMove.py --baud
  delay(500);              // Short stabilization time
  // USB write mutex — must exist before motion::init() launches tasks that write to USBSerial
  g_usbWriteMutex = xSemaphoreCreateMutex();
  Serial1.println("\r\n[SERIAL] Ready");

  // Reset reason and boot counter
  esp_reset_reason_t reason = esp_reset_reason();
  preferences.begin("system", false);
  bootCount = preferences.getUInt("boot_count", 0);
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
  homing::init();
  sendSettingsPacket(); // send initial SETTINGS packet so GUI can populate panel

  Serial1.println("Setup complete");
}

/// @brief Send a binary SETTINGS packet (0xAA 0xEE, 25 bytes) over USBSerial.
/// Safe to call from setup() and processSerialCommands() (both run when
/// TelemetryTask is idle).  g_usbWriteMutex serialises against DiagnosticsTask.
void sendSettingsPacket() {
  uint8_t buf[25];
  buf[0] = 0xAA; buf[1] = 0xEE;
  buf[2] = (uint8_t)setVoltage;
  buf[3] = (uint8_t)setCurrent;
  buf[4] = (uint8_t)setHoldCurrent;
  buf[5] = (uint8_t)setHoldDelay;
  uint16_t ms = (uint16_t)setMicrosteps;
  buf[6] = ms & 0xFF; buf[7] = ms >> 8;
  buf[8] = (uint8_t)setStall;
  uint8_t ssm = 0;
  if      (strcmp(standstillMode, "FREEWHEELING")   == 0) ssm = 1;
  else if (strcmp(standstillMode, "BRAKING")        == 0) ssm = 2;
  else if (strcmp(standstillMode, "STRONG_BRAKING") == 0) ssm = 3;
  buf[9]  = ssm;
  buf[10] = stealthchopEnabled ? 1 : 0;
  buf[11] = coolstepEnabled    ? 1 : 0;
  // Clamp float gains to uint16_t range before casting to prevent UB and
  // silent wrap-around (e.g. Kv=1.0 would overflow the x100000 encoding).
  float kpClamped = setKp < 0.0f ? 0.0f : (setKp > 65.535f   ? 65.535f   : setKp);
  float kdClamped = setKd < 0.0f ? 0.0f : (setKd > 6.5535f   ? 6.5535f   : setKd);
  float kvClamped = setKv < 0.0f ? 0.0f : (setKv > 0.65535f  ? 0.65535f  : setKv);
  uint16_t kpInt = (uint16_t)(kpClamped * 1000.0f);
  buf[12] = kpInt & 0xFF; buf[13] = kpInt >> 8;
  uint16_t kdInt = (uint16_t)(kdClamped * 10000.0f);
  buf[14] = kdInt & 0xFF; buf[15] = kdInt >> 8;
  uint16_t kvInt = (uint16_t)(kvClamped * 100000.0f);
  buf[16] = kvInt & 0xFF; buf[17] = kvInt >> 8;
  uint16_t spd = (uint16_t)setSpreadCycleSpeed;
  buf[18] = spd & 0xFF; buf[19] = spd >> 8;
  float daClamped = setDAlpha < 0.0f ? 0.0f : (setDAlpha > 6.5535f ? 6.5535f : setDAlpha);
  uint16_t daInt = (uint16_t)(daClamped * 10000.0f);
  buf[20] = daInt & 0xFF; buf[21] = daInt >> 8;
  float jerkClamped = setJerk < 0.0f ? 0.0f : (setJerk > 65535.0f ? 65535.0f : setJerk);
  uint16_t jerkInt = (uint16_t)jerkClamped;  // ramp time in ms, 1 ms resolution
  buf[22] = jerkInt & 0xFF; buf[23] = jerkInt >> 8;
  uint8_t cs = 0;
  for (int i = 2; i < 24; i++) cs ^= buf[i];
  buf[24] = cs;
  UsbWriteGuard guard;
  if (guard) USBSerial.write(buf, 25);
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
            bool chain = doc["chain"] | false;
            Serial1.printf(
                "%s Command - Target/Dist: %ld, Accel: %.2f, Speed: %.2f, Chain: %s\n",
                isAbs ? "Absolute" : "Relative", dist, accel, speed, chain ? "yes" : "no");
            if (!motion::addCommand(dist, accel, speed, isAbs, chain)) {
              Serial1.println("ERR: motion queue full, command dropped");
            }

          } else if (strcmp(cmd, "set_phase_lead") == 0) {
            float kv = doc["kv"] | 0.0f;
            setKv = kv;
            motion::setPhaseLeadGain(kv);
            Serial1.printf("Set phase lead gain Kv: %.4f\n", kv);

          } else if (strcmp(cmd, "set_pd") == 0) {
            float kp = doc["kp"] | 3.0f;
            float kd = doc["kd"] | 0.1f;
            float da = doc["d_alpha"] | setDAlpha;  // keep current if not supplied
            setKp = kp; setKd = kd; setDAlpha = da;
            motion::setPD(kp, kd);
            motion::setDFilterAlpha(da);
            Serial1.printf("Set PD - Kp: %.4f, Kd: %.4f, d_alpha: %.4f\n", kp, kd, da);

          } else if (strcmp(cmd, "set_jerk") == 0) {
            float ms = doc["value"] | 0.0f;
            setJerk = ms;
            motion::setJerkRampTime(ms / 1000.0f);
            Serial1.printf("Set jerk ramp time: %.0f ms\n", ms);

          } else if (strcmp(cmd, "set_pid") == 0) {
            // Legacy alias: map ki → kd for backwards compat with scripts
            float kp = doc["kp"] | 3.0f;
            float kd = doc["kd"] | doc["ki"] | 0.1f;
            setKp = kp; setKd = kd;
            motion::setPD(kp, kd);
            Serial1.printf("Set PD (legacy set_pid) - Kp: %.4f, Kd: %.4f\n", kp, kd);

          } else if (strcmp(cmd, "set_voltage") == 0) {
            if (motion::isRunning()) {
              Serial1.printf("ERR: '%s' rejected — motion in progress\n", cmd);
            } else {
              setVoltage = doc["value"] | 20;
              // Update CH224K CFG pins only — TMC registers don't need reprogramming for a voltage change
              if      (setVoltage == 5)  { digitalWrite(PD_CFG1, HIGH); digitalWrite(PD_CFG2, LOW);  digitalWrite(PD_CFG3, LOW);  }
              else if (setVoltage == 9)  { digitalWrite(PD_CFG1, LOW);  digitalWrite(PD_CFG2, LOW);  digitalWrite(PD_CFG3, LOW);  }
              else if (setVoltage == 12) { digitalWrite(PD_CFG1, LOW);  digitalWrite(PD_CFG2, LOW);  digitalWrite(PD_CFG3, HIGH); }
              else if (setVoltage == 15) { digitalWrite(PD_CFG1, LOW);  digitalWrite(PD_CFG2, HIGH); digitalWrite(PD_CFG3, HIGH); }
              else if (setVoltage == 20) { digitalWrite(PD_CFG1, LOW);  digitalWrite(PD_CFG2, HIGH); digitalWrite(PD_CFG3, LOW);  }
              motion::setConfiguredVoltage((float)setVoltage);
              Serial1.printf("Set voltage: %d V\n", setVoltage);
            }

          } else if (strcmp(cmd, "set_current") == 0) {
            if (motion::isRunning()) {
              Serial1.printf("ERR: '%s' rejected — motion in progress\n", cmd);
            } else {
              setCurrent = doc["value"] | 50;
              tmc::setRunCurrent(setCurrent);
              Serial1.printf("Set current: %d%%\n", setCurrent);
            }

          } else if (strcmp(cmd, "set_hold_current") == 0) {
            if (motion::isRunning()) {
              Serial1.printf("ERR: '%s' rejected — motion in progress\n", cmd);
            } else {
              setHoldCurrent = doc["value"] | 25;
              tmc::setHoldCurrent(setHoldCurrent);
              Serial1.printf("Set hold current: %d%%\n", setHoldCurrent);
            }

          } else if (strcmp(cmd, "set_hold_delay") == 0) {
            if (motion::isRunning()) {
              Serial1.printf("ERR: '%s' rejected — motion in progress\n", cmd);
            } else {
              setHoldDelay = doc["value"] | 8;
              tmc::setHoldDelay(setHoldDelay);
              Serial1.printf("Set hold delay: %d\n", setHoldDelay);
            }

          } else if (strcmp(cmd, "set_microsteps") == 0) {
            if (motion::isRunning()) {
              Serial1.printf("ERR: '%s' rejected — motion in progress\n", cmd);
            } else {
              setMicrosteps = doc["value"] | 32;
              tmc::setMicrostepsPerStep(setMicrosteps);
              motion::setMicrosteps(setMicrosteps);
              Serial1.printf("Set microsteps: %d\n", setMicrosteps);
            }

          } else if (strcmp(cmd, "set_stall_threshold") == 0) {
            if (motion::isRunning()) {
              Serial1.printf("ERR: '%s' rejected — motion in progress\n", cmd);
            } else {
              setStall = doc["value"] | 10;
              tmc::setStallGuardThreshold(setStall);
              Serial1.printf("Set stall threshold: %d\n", setStall);
            }

          } else if (strcmp(cmd, "set_standstill_mode") == 0) {
            if (motion::isRunning()) {
              Serial1.printf("ERR: '%s' rejected — motion in progress\n", cmd);
            } else {
              const char *mode = doc["value"] | "NORMAL";
              strncpy(standstillMode, mode, sizeof(standstillMode) - 1);
              standstillMode[sizeof(standstillMode) - 1] = '\0';
              uint8_t ssm = 0;
              if      (strcmp(standstillMode, "FREEWHEELING")   == 0) ssm = 1;
              else if (strcmp(standstillMode, "BRAKING")        == 0) ssm = 2;
              else if (strcmp(standstillMode, "STRONG_BRAKING") == 0) ssm = 3;
              tmc::setStandstillMode(ssm);
              Serial1.printf("Set standstill mode: %s\n", standstillMode);
            }

          } else if (strcmp(cmd, "set_spread_cycle_speed") == 0) {
            if (motion::isRunning()) {
              Serial1.printf("ERR: '%s' rejected — motion in progress\n", cmd);
            } else {
              setSpreadCycleSpeed = doc["value"] | 0;
              // TSTEP measures 1/256-microstep period, not input-step period.
              // Scale: TPWMTHRS = fCLK * microsteps / (256 * velocity)
              uint32_t tpwm = 0;
              if (setSpreadCycleSpeed > 0) {
                tpwm = (uint32_t)((12000000ULL * (uint32_t)setMicrosteps) / (256ULL * (uint32_t)setSpreadCycleSpeed));
              }
              tmc::setStealthChopThreshold(tpwm);
              Serial1.printf("Set SpreadCycle speed: %d steps/s (TPWMTHRS=%lu)\n", setSpreadCycleSpeed, (unsigned long)tpwm);
            }

          } else if (strcmp(cmd, "set_stealthchop") == 0) {
            if (motion::isRunning()) {
              Serial1.printf("ERR: '%s' rejected — motion in progress\n", cmd);
            } else {
              stealthchopEnabled = (int)(doc["value"] | 1) != 0;
              if (stealthchopEnabled) tmc::enableStealthChop();
              else                    tmc::disableStealthChop();
              Serial1.printf("Set StealthChop: %s\n", stealthchopEnabled ? "ON" : "OFF");
            }

          } else if (strcmp(cmd, "set_coolstep") == 0) {
            if (motion::isRunning()) {
              Serial1.printf("ERR: '%s' rejected — motion in progress\n", cmd);
            } else {
              coolstepEnabled = (int)(doc["value"] | 1) != 0;
              if (coolstepEnabled) tmc::enableCoolStep(1, 0);
              else                 tmc::disableCoolStep();
              Serial1.printf("Set CoolStep: %s\n", coolstepEnabled ? "ON" : "OFF");
            }

          } else if (strcmp(cmd, "estop") == 0) {
            motion::triggerEstop();
            Serial1.println("E-STOP triggered via serial");

          } else if (strcmp(cmd, "home") == 0) {
            if (motion::isRunning()) {
              Serial1.println("ERR: 'home' rejected — motion in progress");
            } else {
              homing::HomingParams hp;
              hp.directionCW      = strcmp(doc["direction"] | "cw", "cw") == 0;
              hp.currentPct       = doc["current_pct"] | 65;
              hp.speed1           = doc["speed1"] | 3000.0f;
              hp.speed2           = doc["speed2"] | 500.0f;
              hp.sgThresh1        = doc["sg_thresh1"] | 65;
              hp.sgThresh2        = doc["sg_thresh2"] | 10;
              hp.backoffSteps     = doc["backoff"] | 3200.0f;
              hp.timeoutMs        = doc["timeout_ms"] | 10000;
              // Fill restore settings from current main.cpp statics
              hp.restoreCurrent     = setCurrent;
              hp.restoreSgThresh    = setStall;
              hp.restoreStealthchop = stealthchopEnabled;
              hp.restoreCoolstep    = coolstepEnabled;
              if (!homing::start(hp)) {
                Serial1.println("ERR: homing already in progress");
              } else {
                Serial1.println("Homing started");
              }
            }

          } else if (strcmp(cmd, "reset_position") == 0) {
            if (motion::isRunning()) {
              Serial1.println("ERR: 'reset_position' rejected — motion in progress");
            } else {
              encoder::resetPosition();
              motion::reZero();
              Serial1.println("Position reset to zero");
            }

          } else if (strcmp(cmd, "save") == 0) {
            writeSettings();
            Serial1.println("Settings saved to flash");

          } else if (strcmp(cmd, "get_settings") == 0) {
            Serial1.printf(
                "{\"voltage\":%d,\"current\":%d,\"hold_current\":%d,"
                "\"hold_delay\":%d,\"microsteps\":%d,"
                "\"stall_threshold\":%d,\"standstill_mode\":\"%s\","
                "\"stealthchop\":%s,\"coolstep\":%s,"
                "\"kp\":%.4f,\"kd\":%.4f,\"kv\":%.4f,\"d_alpha\":%.4f,\"jerk\":%.1f}\n",
                setVoltage, setCurrent, setHoldCurrent, setHoldDelay,
                setMicrosteps, setStall, standstillMode,
                stealthchopEnabled ? "true" : "false",
                coolstepEnabled    ? "true" : "false",
                setKp, setKd, setKv, setDAlpha, setJerk);
            sendSettingsPacket();

          } else if (strcmp(cmd, "get_driver_status") == 0) {
            tmc::DriverStatus ds   = tmc::getDriverStatus();
            tmc::DriverSettings cfg = tmc::getDriverSettings();
            uint16_t pwmScale      = tmc::getPwmScaleSum();
            uint32_t tstep         = tmc::getInterstepDuration();
            Serial1.printf(
                "{\"cs_actual\":%u,\"standstill\":%s,\"stealth_chop\":%s,"
                "\"ot_warn\":%s,\"ot_shutdown\":%s,"
                "\"irun\":%u,\"ihold\":%u,\"iholddelay\":%u,"
                "\"cool_step\":%s,\"auto_scaling\":%s,"
                "\"pwm_scale\":%u,\"tstep\":%lu,"
                "\"hold_active\":%s,\"hold_target\":%.1f}\n",
                ds.current_scaling,
                ds.standstill ? "true" : "false",
                ds.stealth_chop ? "true" : "false",
                ds.over_temperature_warning ? "true" : "false",
                ds.over_temperature_shutdown ? "true" : "false",
                cfg.irun_percent, cfg.ihold_percent, cfg.iholddelay_percent,
                cfg.cool_step_enabled ? "true" : "false",
                cfg.automatic_current_scaling ? "true" : "false",
                pwmScale, (unsigned long)tstep,
                motion::isHoldActive() ? "true" : "false",
                motion::getHoldTarget());

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
    // TMC UART reads and hold-state diagnostics are handled by DiagnosticsTask
    // (Core 0) to keep Core 1 loop() free of UART traffic.
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
  tmc::setHoldCurrent(setHoldCurrent);
  tmc::setHoldDelay(setHoldDelay);
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

  // Apply StealthChop and CoolStep mode settings
  if (stealthchopEnabled) tmc::enableStealthChop();
  else                    tmc::disableStealthChop();
  if (coolstepEnabled) tmc::enableCoolStep(1, 0);
  else                 tmc::disableCoolStep();

  // Apply SpreadCycle speed threshold (TPWMTHRS register)
  // TSTEP measures 1/256-microstep period, not input-step period.
  // Scale: TPWMTHRS = fCLK * microsteps / (256 * velocity)
  uint32_t tpwm = 0;
  if (setSpreadCycleSpeed > 0) {
    tpwm = (uint32_t)((12000000ULL * (uint32_t)setMicrosteps) / (256ULL * (uint32_t)setSpreadCycleSpeed));
  }
  tmc::setStealthChopThreshold(tpwm);
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
    setCurrent    = preferences.getInt("current",        60);
    setHoldCurrent= preferences.getInt("holdCurrent",   25);
    setHoldDelay  = preferences.getInt("holdDelay",       8);
    setStall      = preferences.getInt("stallThreshold", 10);
    String mode   = preferences.getString("standstillMode", "NORMAL");
    strncpy(standstillMode, mode.c_str(), sizeof(standstillMode) - 1);
    standstillMode[sizeof(standstillMode) - 1] = '\0';
    stealthchopEnabled = preferences.getBool("stealthChop", true);
    coolstepEnabled    = preferences.getBool("coolStep",    true);
    setSpreadCycleSpeed = preferences.getInt("spreadSpeed", 0);
    setKp     = preferences.getFloat("kp",      3.0f);
    setKd     = preferences.getFloat("kd",      0.1f);
    setKv     = preferences.getFloat("kv",      0.0f);
    setDAlpha = preferences.getFloat("d_alpha", 0.8f);
    setJerk   = preferences.getFloat("jerk_ms", 0.0f);
    preferences.end();
    // Apply PD gains now (motion tasks pick them up on start)
    motion::setPD(setKp, setKd);
    motion::setPhaseLeadGain(setKv);
    motion::setDFilterAlpha(setDAlpha);
    if (setJerk > 0.0f) {
        motion::setJerkRampTime(setJerk / 1000.0f);
    }
  }
}

void writeSettings() {
  preferences.begin("settings", false);
  preferences.putInt("voltage",        setVoltage);
  preferences.putInt("microsteps",     setMicrosteps);
  preferences.putInt("current",        setCurrent);
  preferences.putInt("holdCurrent",    setHoldCurrent);
  preferences.putInt("holdDelay",      setHoldDelay);
  preferences.putInt("stallThreshold", setStall);
  preferences.putString("standstillMode", standstillMode);
  preferences.putBool("stealthChop",   stealthchopEnabled);
  preferences.putBool("coolStep",      coolstepEnabled);
  preferences.putInt("spreadSpeed",    setSpreadCycleSpeed);
  preferences.putFloat("kp",           setKp);
  preferences.putFloat("kd",           setKd);
  preferences.putFloat("kv",           setKv);
  preferences.putFloat("d_alpha",      setDAlpha);
  preferences.putFloat("jerk_ms",      setJerk);
  Serial1.println("Saving settings to flash");
  preferences.end();
  configureSettings();
}
