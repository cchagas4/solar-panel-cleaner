
#ifndef ARDUINO_SERVOMOTOR_H
#define ARDUINO_SERVOMOTOR_H
#include <Arduino.h>
#include <Servo.h>

class ServoMotor
{
private:
	Servo servoMotor;
public:
	ServoMotor();
	ServoMotor(byte servoPin);

	void write(int degree);
};
#endif
