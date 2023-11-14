#ifndef ARDUINO_OPERATION_H
#define ARDUINO_OPERATION_H
#include <Arduino.h>
#include "Led.h"
#include "Motor.h"
#include "UltrassonicSensor.h"

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
	UltrassonicSensor frontSensor;
	UltrassonicSensor backSensor;

	int maxDistance = 50;
	void distancePrint(int distance);

public:
	Operation();
	void configLeds(Led onoff, Led cleaning, Led squeegeeing);
	void configMotors(Motor engine, Motor brush, Motor valve);
	void configUltrassonicSensors(UltrassonicSensor front, UltrassonicSensor back);
	inline static int status = 0;
	void control();
};
#endif