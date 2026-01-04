#pragma once
#include <Arduino.h>

// Initialize TMC driver with RX/TX pins
void tmc_init(int rx_pin, int tx_pin);

void tmc_setRunCurrent(int percent);
void tmc_enableAutomaticCurrentScaling();
void tmc_enableStealthChop();
void tmc_setCoolStepDurationThreshold(int ms);
void tmc_disable();
void tmc_enable();
void tmc_moveAtVelocity(int v);
int tmc_getStallGuardResult();
String tmc_getStatusString();
bool tmc_hardwareDisabled();
void tmc_setMicrostepsPerStep(int ms);
void tmc_setStallGuardThreshold(int th);
void tmc_setStandstillMode(int mode);
