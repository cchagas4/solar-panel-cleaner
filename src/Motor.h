
#ifndef ARDUINO_MOTOR_H
#define ARDUINO_MOTOR_H
#include <Arduino.h>

class Motor
{
private:
	const int MIN_SPEED = 0;
	const int MAX_SPEED = 255;

	byte forwardMotorPin;
	byte backwardMotorPin;
	byte actuator;

	bool hasSpeedControl;

public:
	Motor(byte forwardMotorPin);
	Motor(byte forwardMotorPin, byte backwardMotorPin);

	void moveForward(int speed);
	void moveBackward(int speed);
	void stop();
	void start();
	void startSmooth(bool isForward);
};
#endif
