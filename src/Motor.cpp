#include "Motor.h"

Motor::Motor(byte forwardMotorPin)
{
    this->hasSpeedControl = false;
    this->actuator = forwardMotorPin;
    pinMode(forwardMotorPin, OUTPUT);
}

Motor::Motor(byte forwardMotorPin, byte backwardMotorPin)
{
    this->forwardMotorPin = forwardMotorPin;
    this->backwardMotorPin = forwardMotorPin;

    pinMode(forwardMotorPin, OUTPUT);
    pinMode(backwardMotorPin, OUTPUT);

    this->hasSpeedControl = true;
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
    if (hasSpeedControl)
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
    if (hasSpeedControl)
    {
        analogWrite(forwardMotorPin, MAX_SPEED);
        analogWrite(backwardMotorPin, MAX_SPEED);
    }
    else
    {
        digitalWrite(actuator, LOW);
    }
}

void Motor::startSmooth(bool isForward)
{
    int speed = 0;
    while (digitalRead(forwardMotorPin) != MAX_SPEED)
    {
        if (isForward)
        {
            moveForward(speed);
        }
        else
        {
            moveBackward(speed);
        }

        delay(50);
        speed++;
    }
}