
#ifndef ARDUINO_POWER_H
#define ARDUINO_POWER_H
#include <Arduino.h>
#include "Operation.h"

class Power
{
private:
	byte pins[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
	bool isOn;
	byte onOff;
	byte emergency;
	// Led indecativeLed;

public:
	Power(byte power, byte emergency);

	void powerOn();
	void powerOff();
};
#endif
