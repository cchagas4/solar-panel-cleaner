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
    analogWrite(forwardMotorPin, speed);
    analogWrite(backwardMotorPin, MIN_SPEED);
}
void Motor::moveBackward(int speed)
{
    analogWrite(backwardMotorPin, speed);
    analogWrite(forwardMotorPin, MIN_SPEED);
}

void Motor::stop()
{
    digitalWrite(actuator, LOW);
}

void Motor::start()
{
    digitalWrite(actuator, HIGH);
}