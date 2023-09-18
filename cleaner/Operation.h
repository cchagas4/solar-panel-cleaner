#ifndef ARDUINO_OPERATION_H
#define ARDUINO_OPERATION_H
#include <Arduino.h>
#include "Led.h"
#include "Motor.h"

class Operation
{
private:
	Led onoffLed;
	Led cleaningLed;
	Led squeegeeingLed;
	Motor engineMotor;
	Motor brushMotor;
	Motor valveMotor;
	String statusDescription;

public:
	Operation();
	void configLeds(Led onoff, Led cleaning, Led squeegeeing);
	void configMotors(Motor engine, Motor brush, Motor valve);
	inline static int status = 0;
	void control();
};
#endif