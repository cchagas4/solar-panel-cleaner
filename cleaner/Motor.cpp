#include "Motor.h"

Motor::Motor()
{
}

Motor::Motor(byte actuator)
{
    this->hasDirection = false;
    this->actuator = actuator;
    pinMode(actuator, OUTPUT);
}

Motor::Motor(byte forwardMotorPin, byte backwardMotorPin)
{
    this->forwardMotorPin = forwardMotorPin;
    this->backwardMotorPin = backwardMotorPin;

    pinMode(forwardMotorPin, OUTPUT);
    pinMode(backwardMotorPin, OUTPUT);

    this->hasDirection = true;
}

void Motor::moveForward(int speed)
{
    analogWrite(backwardMotorPin, MIN_SPEED);
    analogWrite(forwardMotorPin, speed);
}

void Motor::moveBackward(int speed)
{
    analogWrite(forwardMotorPin, MIN_SPEED);
    analogWrite(backwardMotorPin, speed);
}

void Motor::stop()
{
    digitalWrite(actuator, LOW);
}

void Motor::start()
{
    digitalWrite(actuator, HIGH);
}