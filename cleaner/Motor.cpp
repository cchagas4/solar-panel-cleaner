#include "Motor.h"

Motor::Motor(byte actuator)
{
    hasSpeedControl = false;
    actuator = actuator;
    pinMode(actuator, OUTPUT);
}

Motor::Motor(byte forwardMotorPin, byte backwardMotorPin)
{
    forwardMotorPin = forwardMotorPin;
    backwardMotorPin = forwardMotorPin;

    pinMode(forwardMotorPin, OUTPUT);
    pinMode(backwardMotorPin, OUTPUT);

    hasSpeedControl = true;
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