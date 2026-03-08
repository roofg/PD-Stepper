#include "tmc_driver.h"
#include <HardwareSerial.h>
#include <TMC2209.h>

namespace tmc {
static TMC2209 stepper_driver;
static HardwareSerial serial_stream(2);
static const long SERIAL_BAUD_RATE = 115200;

// Pin definitions from main.cpp (should ideally be in a common header)
#define STEP_PIN 5
#define DIR_PIN 6

void init(int rx_pin, int tx_pin) {
  serial_stream.begin(SERIAL_BAUD_RATE, SERIAL_8N1, rx_pin, tx_pin);
  stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE,
                       TMC2209::SERIAL_ADDRESS_0, rx_pin, tx_pin);
}

/**
 * @brief Set the run current in percent of maximum current.
 *
 * @param percent
 */
void setRunCurrent(int percent) { stepper_driver.setRunCurrent(percent); }

/**
 * @brief Set the hold current in percent of maximum current.
 *
 * @param percent
 */
void setHoldCurrent(int percent) { stepper_driver.setHoldCurrent(percent); }

/**
 * @brief Enable automatic current scaling.
 *
 */
void enableAutomaticCurrentScaling() {
  stepper_driver.enableAutomaticCurrentScaling();
}

/**
 * @brief Enable stealth chop.
 *
 */
void enableStealthChop() { stepper_driver.enableStealthChop(); }

/**
 * @brief Disable stealth chop.
 *
 */
void disableStealthChop() { stepper_driver.disableStealthChop(); }

/**
 * @brief Set the cool step duration threshold in ms.
 *
 * @param ms
 */
void setCoolStepDurationThreshold(int ms) {
  stepper_driver.setCoolStepDurationThreshold(ms);
}

/**
 * @brief Disable the driver.
 *
 */
void disable() { stepper_driver.disable(); }

/**
 * @brief Enable the driver.
 *
 */
void enable() { stepper_driver.enable(); }

/**
 * @brief Move at velocity.
 *
 * @param v
 */
void moveAtVelocity(int v) {
  // We keep this for backward compatibility or legacy use,
  // but the new motion system will use step() pulses.
  stepper_driver.moveAtVelocity(v);
}

void step() {
    // Legacy single-step function. New architecture uses stepgen:: ISR.
    // Kept for compatibility; not called during normal operation.
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEP_PIN, LOW);
}

void setDirection(bool forward) {
  digitalWrite(DIR_PIN, forward ? LOW : HIGH); // Match polarity if needed
}

/**
 * @brief Get the stall guard result.
 *
 * @return int
 */
int getStallGuardResult() { return stepper_driver.getStallGuardResult(); }

String getStatusString() {
  TMC2209::Status status = stepper_driver.getStatus();
  if (status.over_temperature_warning)
    return String("Over Temp Warning");
  if (status.over_temperature_shutdown)
    return String("Over Temp Shutdown");
  return String("No Errors");
}

bool hardwareDisabled() { return stepper_driver.hardwareDisabled(); }

void setMicrostepsPerStep(int ms) { stepper_driver.setMicrostepsPerStep(ms); }
void setStallGuardThreshold(int th) {
  stepper_driver.setStallGuardThreshold(th);
}
void setStandstillMode(int mode) {
  // map simple integer modes to driver constants if needed
  switch (mode) {
  case 1:
    stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);
    break;
  case 2:
    stepper_driver.setStandstillMode(stepper_driver.BRAKING);
    break;
  case 3:
    stepper_driver.setStandstillMode(stepper_driver.STRONG_BRAKING);
    break;
  default:
    stepper_driver.setStandstillMode(stepper_driver.NORMAL);
    break;
  }
}
} // namespace tmc
