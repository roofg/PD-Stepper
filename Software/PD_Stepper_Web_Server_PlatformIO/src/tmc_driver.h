#pragma once
#include <Arduino.h>

namespace tmc {
// Initialize TMC driver with RX/TX pins
void init(int rx_pin, int tx_pin);

void setRunCurrent(int percent);
void setHoldCurrent(int percent);
void enableAutomaticCurrentScaling();
void enableStealthChop();
void disableStealthChop();
void setCoolStepDurationThreshold(int ms);
void disable();
void enable();
int getStallGuardResult();
String getStatusString();
bool hardwareDisabled();
void setMicrostepsPerStep(int ms);
void setStallGuardThreshold(int th);
void setStandstillMode(int mode);
} // namespace tmc