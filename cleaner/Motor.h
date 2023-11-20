
#ifndef ARDUINO_MOTOR_H
#define ARDUINO_MOTOR_H
#include <Arduino.h>

class Motor
{
private:
	const static int MIN_SPEED = 0;
	const static int MAX_SPEED = 255;

	byte forwardMotorPin;
	byte backwardMotorPin;
	byte actuator;

	bool hasDirection;

public:
	Motor();
	Motor(byte actuator);
	Motor(byte forwardMotorPin, byte backwardMotorPin);

	void moveForward(int speed);
	void moveBackward(int speed);
	void stop();
	void start();
};
#endif
