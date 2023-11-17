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
    frontDistance = frontSensor.getUltrasonicDistance();
    backDistance = backSensor.getUltrasonicDistance();
    Serial.println("[CONTROL] | FRONT DISTANCE --> " + String(frontDistance));
    Serial.println("[CONTROL] | BACK DISTANCE --> " + String(backDistance));

    switch (Operation::status)
    {

    case 0: // "OFF"
        statusDescription = "OFF";
        ledsControl();
        turnOffMotors();
        break;
    case 1: // "ON" TODO
        statusDescription = "ON";
        ledsControl();
        // start cleaning
        status = 2;
        break;
    case 2: // "CLEANING"
        statusDescription = "CLEANING";
        ledsControl();
        squeegeeRight.write(0);
        squeegeeLeft.write(0);

        if (maxDistance < frontDistance)
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
        ledsControl();
        squeegeeRight.write(180);
        squeegeeLeft.write(180);

        if (maxDistance < backDistance)
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
        turnOffMotors();
        break;
    default:
        Operation::status = 4;
        break;
    }
    Serial.println("[status] | " + statusDescription + " |");
}

void Operation::turnOffMotors()
{
    engineMotor.stop();
    brushMotor.stop();
    valveMotor.stop();
}

void Operation::ledsControl()
{
    switch (Operation::status)
    {
    case 0: // "OFF"
        onoffLed.blink();
        cleaningLed.turnOff();
        squeegeeingLed.turnOff();
        break;
    case 1: // "ON" TODO
        onoffLed.turnOn();
        cleaningLed.turnOff();
        squeegeeingLed.turnOff();
        break;
    case 2: // "CLEANING"
        cleaningLed.fade();
        squeegeeingLed.turnOff();
        break;
    case 3: // "SQUEEGEEING"
        cleaningLed.turnOff();
        squeegeeingLed.blink();
        break;
    case 4: // "ERROR"
        onoffLed.blink();
        cleaningLed.blink();
        squeegeeingLed.blink();
        break;
    }
}
