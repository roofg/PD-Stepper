#include "tmc_driver.h"
#include "pins.h"
#include <HardwareSerial.h>
#include <TMC2209.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace tmc {
static TMC2209 stepper_driver;
static HardwareSerial serial_stream(2);
static const long SERIAL_BAUD_RATE = 115200;

// Mutex serialising all stepper_driver UART accesses.
// Both PlannerTask (Core 0, pri 5) and loopTask (Core 0, pri 1) use the driver;
// without a mutex, PlannerTask can preempt loopTask mid-transaction and corrupt
// the UART request-response cycle.
static SemaphoreHandle_t s_mutex = nullptr;

// RAII guard: takes the mutex on construction, gives it back on destruction.
// Timeout is generous (10 ms) — a TMC UART transaction takes ~1 ms at 115200.
struct TmcLock {
    bool held;
    TmcLock() : held(s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {}
    ~TmcLock() { if (held) xSemaphoreGive(s_mutex); }
    explicit operator bool() const { return held; }
};

void init(int rx_pin, int tx_pin) {
  s_mutex = xSemaphoreCreateMutex();
  serial_stream.begin(SERIAL_BAUD_RATE, SERIAL_8N1, rx_pin, tx_pin);
  stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE,
                       TMC2209::SERIAL_ADDRESS_0, rx_pin, tx_pin);
}

void setRunCurrent(int percent)    { TmcLock g; if (g) stepper_driver.setRunCurrent(percent); }
void setHoldCurrent(int percent)   { TmcLock g; if (g) stepper_driver.setHoldCurrent(percent); }
void enableAutomaticCurrentScaling()     { TmcLock g; if (g) stepper_driver.enableAutomaticCurrentScaling(); }
void enableAutomaticGradientAdaptation() { TmcLock g; if (g) stepper_driver.enableAutomaticGradientAdaptation(); }
void enableStealthChop()           { TmcLock g; if (g) stepper_driver.enableStealthChop(); }
void disableStealthChop()          { TmcLock g; if (g) stepper_driver.disableStealthChop(); }
void setCoolStepDurationThreshold(int ms) { TmcLock g; if (g) stepper_driver.setCoolStepDurationThreshold(ms); }

void enableCoolStep(uint8_t lower, uint8_t upper) {
  // lower (SEMIN): CoolStep boosts current when SG result < lower*32. Range 1–15.
  // upper (SEMAX): CoolStep reduces current when SG result >= (SEMIN+SEMAX+1)*32.
  //                Range 0–15; library comment recommends 0–2.
  TmcLock g; if (g) stepper_driver.enableCoolStep(lower, upper);
}

void disableCoolStep()             { TmcLock g; if (g) stepper_driver.disableCoolStep(); }
void setPowerDownDelay(uint8_t delay) { TmcLock g; if (g) stepper_driver.setPowerDownDelay(delay); }
void disable()                     { TmcLock g; if (g) stepper_driver.disable(); }
void enable()                      { TmcLock g; if (g) stepper_driver.enable(); }

void moveAtVelocity(int v)         { TmcLock g; if (g) stepper_driver.moveAtVelocity(v); }

int getStallGuardResult() {
  TmcLock g;
  return g ? stepper_driver.getStallGuardResult() : 0;
}

String getStatusString() {
  TmcLock g;
  if (!g) return String("Mutex timeout");
  TMC2209::Status status = stepper_driver.getStatus();
  if (status.over_temperature_warning)  return String("Over Temp Warning");
  if (status.over_temperature_shutdown) return String("Over Temp Shutdown");
  return String("No Errors");
}

bool hardwareDisabled() {
  TmcLock g;
  return g ? stepper_driver.hardwareDisabled() : false;
}

void setMicrostepsPerStep(int ms)  { TmcLock g; if (g) stepper_driver.setMicrostepsPerStep(ms); }
void setStallGuardThreshold(int th) { TmcLock g; if (g) stepper_driver.setStallGuardThreshold(th); }

void setStandstillMode(int mode) {
  TmcLock g;
  if (!g) return;
  switch (mode) {
  case 1:  stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);    break;
  case 2:  stepper_driver.setStandstillMode(stepper_driver.BRAKING);         break;
  case 3:  stepper_driver.setStandstillMode(stepper_driver.STRONG_BRAKING);  break;
  default: stepper_driver.setStandstillMode(stepper_driver.NORMAL);          break;
  }
}

void setHoldDelay(int percent)     { TmcLock g; if (g) stepper_driver.setHoldDelay(percent); }
void setAllCurrentValues(int run, int hold, int delay) {
  TmcLock g; if (g) stepper_driver.setAllCurrentValues(run, hold, delay);
}

void setStealthChopThreshold(uint32_t tpwmthrs) {
  TmcLock g; if (g) stepper_driver.setStealthChopDurationThreshold(tpwmthrs);
}

// --- Diagnostic reads (safe from any Core 0 context via mutex) ---

DriverStatus getDriverStatus() {
  TmcLock g;
  DriverStatus s = {};
  if (!g) return s;
  TMC2209::Status raw = stepper_driver.getStatus();
  s.current_scaling           = raw.current_scaling;
  s.standstill                = raw.standstill;
  s.stealth_chop              = raw.stealth_chop_mode;
  s.over_temperature_warning  = raw.over_temperature_warning;
  s.over_temperature_shutdown = raw.over_temperature_shutdown;
  s.short_to_ground_a         = raw.short_to_ground_a;
  s.short_to_ground_b         = raw.short_to_ground_b;
  s.open_load_a               = raw.open_load_a;
  s.open_load_b               = raw.open_load_b;
  return s;
}

DriverSettings getDriverSettings() {
  TmcLock g;
  DriverSettings s = {};
  if (!g) return s;
  TMC2209::Settings raw = stepper_driver.getSettings();
  s.irun_percent              = raw.irun_percent;
  s.ihold_percent             = raw.ihold_percent;
  s.iholddelay_percent        = raw.iholddelay_percent;
  s.automatic_current_scaling = raw.automatic_current_scaling_enabled;
  s.cool_step_enabled         = raw.cool_step_enabled;
  return s;
}

uint16_t getPwmScaleSum() {
  TmcLock g;
  return g ? stepper_driver.getPwmScaleSum() : 0;
}

uint32_t getInterstepDuration() {
  TmcLock g;
  return g ? stepper_driver.getInterstepDuration() : 0;
}

} // namespace tmc
