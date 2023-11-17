#include "ServoMotor.h"

ServoMotor::ServoMotor()
{
}

ServoMotor::ServoMotor(byte servoPin)
{
    servoMotor.attach(servoPin);
}

void ServoMotor::write(int degree)
{
    servoMotor.write(degree);
}