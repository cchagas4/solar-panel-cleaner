#include "Operation.h"

Operation::Operation()
{
}

void Operation::configLeds(Led onoff, Led cleaning, Led squeegeeing)
{
    onoffLed = onoff;
    cleaningLed = cleaning;
    squeegeeingLed = squeegeeing;
}

void Operation::configMotors(Motor engine, Motor brush, Motor valve)
{
    engineMotor = engine;
    brushMotor = brush;
    valveMotor = valve;
}

void Operation::configServoMotors(ServoMotor right, ServoMotor left)
{
    squeegeeRight = right;
    squeegeeLeft = left;
}

void Operation::configUltrassonicSensors(UltrassonicSensor front, UltrassonicSensor back)
{
    frontSensor = front;
    backSensor = back;
}

void Operation::control()
{
    switch (Operation::status)
    {

    case 0: // "OFF"
        statusDescription = "OFF";
        onoffLed.turnOn();
        cleaningLed.turnOff();
        squeegeeingLed.turnOff();
        // TODO turn off motors??
        break;
    case 1: // "ON" TODO
        statusDescription = "ON";
        onoffLed.turnOn();
        cleaningLed.blink();
        squeegeeingLed.blink();
        // start cleaning
        status = 2;
        break;
    case 2: // "CLEANING"
        statusDescription = "CLEANING";
        cleaningLed.fade();
        squeegeeingLed.turnOff();
        squeegeeRight.write(0);
        squeegeeLeft.write(0);

        frontDistance = frontSensor.getUltrasonicDistance();
        distancePrint(frontDistance);
        if (maxDistance < frontDistance || frontDistance != 0) // TODO refactor
        {
            engineMotor.moveForward(255);
            brushMotor.start();
            valveMotor.start();
        }
        else
        {
            status = 3;
        }

        break;
    case 3: // "SQUEEGEEING"
        statusDescription = "SQUEEGEEING";
        cleaningLed.turnOff();
        squeegeeingLed.blink();
        squeegeeRight.write(180);
        squeegeeLeft.write(180);

        backDistance = backSensor.getUltrasonicDistance();
        distancePrint(backDistance);
        if (maxDistance < backDistance || backDistance != 0) // TODO refactor
        {
            engineMotor.moveBackward(255);
            brushMotor.stop();
            valveMotor.stop();
        }
        else
        {
            status = 0;
        }
        break;
    case 4: // "ERROR"
        statusDescription = "ERROR";
        cleaningLed.turnOn();
        squeegeeingLed.turnOn();
        engineMotor.stop();
        brushMotor.stop();
        valveMotor.stop();
        break;
    default:
        Operation::status = 4;
        break;
    }
    Serial.println("STATUS ---> [" + statusDescription + "]");
}

void Operation::distancePrint(int distance)
{
    Serial.print("OBSTACLE ---> [");
    Serial.print(distance);
    Serial.println("]");
}
