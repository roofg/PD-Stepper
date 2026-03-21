#pragma once
#include <Arduino.h>

namespace tmc {

// Driver status snapshot (from DRV_STATUS register via UART)
struct DriverStatus {
    uint8_t current_scaling;         // 0–31 actual current scale
    bool    standstill;              // true when no step pulses detected
    bool    stealth_chop;            // true if StealthChop active
    bool    over_temperature_warning;
    bool    over_temperature_shutdown;
};

// Driver settings snapshot (from live config registers)
struct DriverSettings {
    uint8_t irun_percent;
    uint8_t ihold_percent;
    uint8_t iholddelay_percent;
    bool    automatic_current_scaling;
    bool    cool_step_enabled;
};

// Initialize TMC driver with RX/TX pins
void init(int rx_pin, int tx_pin);

void setRunCurrent(int percent);
void setHoldCurrent(int percent);
void enableAutomaticCurrentScaling();
void enableAutomaticGradientAdaptation();
void enableStealthChop();
void disableStealthChop();
void setCoolStepDurationThreshold(int ms);
void enableCoolStep(uint8_t lower = 1, uint8_t upper = 0);
void disableCoolStep();
void setPowerDownDelay(uint8_t delay);
void disable();
void enable();
int  getStallGuardResult();
String getStatusString();
bool hardwareDisabled();
void setMicrostepsPerStep(int ms);
void setStallGuardThreshold(int th);
void setStandstillMode(int mode);
void setHoldDelay(int percent);
void setAllCurrentValues(int run, int hold, int delay);

// Diagnostic reads — call from Core 0 only (uses TMC UART)
DriverStatus  getDriverStatus();
DriverSettings getDriverSettings();
uint16_t getPwmScaleSum();
uint32_t getInterstepDuration();

} // namespace tmc