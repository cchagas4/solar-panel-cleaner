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

void Operation::control()
{
    switch (Operation::status)
    {

    case 0: // "OFF"
        statusDescription = "OFF";
        onoffLed.turnOff();
        cleaningLed.turnOff();
        squeegeeingLed.turnOff();
        break;
    case 1: // "ON" TODO
        statusDescription = "ON";
        onoffLed.turnOn();
        cleaningLed.turnOff();
        squeegeeingLed.turnOff();
        // start cleaning
        status = 2;
        break;
    case 2: // "CLEANING"
        statusDescription = "CLEANING";
        cleaningLed.turnOn();
        squeegeeingLed.turnOff();
        engineMotor.moveForward(255);
        brushMotor.start();
        valveMotor.start();
        break;
    case 3: // "SQUEEGEEING"
        statusDescription = "SQUEEGEEING";
        cleaningLed.turnOff();
        squeegeeingLed.turnOn();
        engineMotor.moveBackward(255);
        brushMotor.stop();
        valveMotor.stop();
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