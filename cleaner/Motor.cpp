#include "Motor.h"

Motor::Motor()
{
}

Motor::Motor(byte actuator)
{
    hasDirection = false;
    this->actuator = actuator;
    pinMode(actuator, OUTPUT);
}

Motor::Motor(byte forwardMotorPin, byte backwardMotorPin)
{
    this->forwardMotorPin = forwardMotorPin;
    this->backwardMotorPin = forwardMotorPin;

    pinMode(forwardMotorPin, OUTPUT);
    pinMode(backwardMotorPin, OUTPUT);

    hasDirection = true;
}

void Motor::moveForward(int speed)
{
    analogWrite(forwardMotorPin, speed);
}
void Motor::moveBackward(int speed)
{
    analogWrite(backwardMotorPin, speed);
}

void Motor::stop()
{
    if (hasDirection)
    {
        analogWrite(forwardMotorPin, MIN_SPEED);
        analogWrite(backwardMotorPin, MIN_SPEED);
    }
    else
    {
        digitalWrite(actuator, LOW);
    }
}

void Motor::start()
{
    if (hasDirection)
    {
        analogWrite(forwardMotorPin, MAX_SPEED);
        analogWrite(backwardMotorPin, MAX_SPEED);
    }
    else
    {
        digitalWrite(actuator, HIGH);
    }
}