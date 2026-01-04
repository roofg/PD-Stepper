#include "tmc_driver.h"
#include <TMC2209.h>
#include <HardwareSerial.h>

static TMC2209 stepper_driver;
static HardwareSerial serial_stream(2);
static const long SERIAL_BAUD_RATE = 115200;

void tmc_init(int rx_pin, int tx_pin){
  serial_stream.begin(SERIAL_BAUD_RATE, SERIAL_8N1, rx_pin, tx_pin);
  stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, rx_pin, tx_pin);
}

void tmc_setRunCurrent(int percent){ stepper_driver.setRunCurrent(percent); }
void tmc_enableAutomaticCurrentScaling(){ stepper_driver.enableAutomaticCurrentScaling(); }
void tmc_enableStealthChop(){ stepper_driver.enableStealthChop(); }
void tmc_setCoolStepDurationThreshold(int ms){ stepper_driver.setCoolStepDurationThreshold(ms); }
void tmc_disable(){ stepper_driver.disable(); }
void tmc_enable(){ stepper_driver.enable(); }
void tmc_moveAtVelocity(int v){ stepper_driver.moveAtVelocity(v); }
int tmc_getStallGuardResult(){ return stepper_driver.getStallGuardResult(); }

String tmc_getStatusString(){
  TMC2209::Status status = stepper_driver.getStatus();
  if (status.over_temperature_warning) return String("Over Temp Warning");
  if (status.over_temperature_shutdown) return String("Over Temp Shutdown");
  return String("No Errors");
}

bool tmc_hardwareDisabled(){ return stepper_driver.hardwareDisabled(); }

void tmc_setMicrostepsPerStep(int ms){ stepper_driver.setMicrostepsPerStep(ms); }
void tmc_setStallGuardThreshold(int th){ stepper_driver.setStallGuardThreshold(th); }
void tmc_setStandstillMode(int mode){
  // map simple integer modes to driver constants if needed
  switch(mode){
    case 1: stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING); break;
    case 2: stepper_driver.setStandstillMode(stepper_driver.BRAKING); break;
    case 3: stepper_driver.setStandstillMode(stepper_driver.STRONG_BRAKING); break;
    default: stepper_driver.setStandstillMode(stepper_driver.NORMAL); break;
  }
}
