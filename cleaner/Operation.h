#ifndef ARDUINO_OPERATION_H
#define ARDUINO_OPERATION_H
#include <Arduino.h>
#include "Led.h"
#include "Motor.h"
#include "ServoMotor.h"
#include "UltrassonicSensor.h"

class Operation
{
private:
	Led onoffLed;
	Led cleaningLed;
	Led squeegeeingLed;
	Motor engineMotor;
	//Motor brushMotor;
	Motor valveMotor;
	//ServoMotor squeegeeRight;
	//ServoMotor squeegeeLeft;
	String statusDescription;
	UltrassonicSensor frontSensor;
	UltrassonicSensor backSensor;

	int maxDistance = 100;
	int frontDistance = 0;
	int backDistance = 0;
	int intervalTIME = 1000;
	unsigned long initTIME = 0;
	unsigned long currentTIME;
	void distancePrint(int distance);

	void turnOffMotors();
	void ledsControl();

public:
	Operation();
	void configLeds(Led onoff, Led cleaning, Led squeegeeing);
	void configMotors(Motor engine, Motor valve);
	//void configServoMotors(ServoMotor right, ServoMotor left);
	void configUltrassonicSensors(UltrassonicSensor front, UltrassonicSensor back);
	inline static int status = 0;
	bool sensorFlag = false;
	void control();
};
#endif