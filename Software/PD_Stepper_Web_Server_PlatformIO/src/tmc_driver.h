#pragma once
#include <Arduino.h>

namespace tmc {
	// Initialize TMC driver with RX/TX pins
	void init(int rx_pin, int tx_pin);

	void setRunCurrent(int percent);
	void enableAutomaticCurrentScaling();
	void enableStealthChop();
	void setCoolStepDurationThreshold(int ms);
	void disable();
	void enable();
	void moveAtVelocity(int v);
	int getStallGuardResult();
	String getStatusString();
	bool hardwareDisabled();
	void setMicrostepsPerStep(int ms);
	void setStallGuardThreshold(int th);
	void setStandstillMode(int mode);
}