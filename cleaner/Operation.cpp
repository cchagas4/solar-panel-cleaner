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

void Operation::configMotors(Motor engine, Motor valve)
{
    engineMotor = engine;
    //brushMotor = brush;
    valveMotor = valve;
}

// void Operation::configServoMotors(ServoMotor right, ServoMotor left)
// {
//     squeegeeRight = right;
//     squeegeeLeft = left;
// }

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

    // if (frontDistance == 0 && backDistance == 0)
    // {
    //     status = 4;
    // }

    switch (Operation::status)
    {
    case 0: // "OFF"
        statusDescription = "OFF";
        ledsControl();
        valveMotor.stop();
        break;

    case 1: // "ON" TODO
        statusDescription = "ON";
        ledsControl();
        valveMotor.start();
        delay(10000); // Waiting for the water to start falling
        status = 2;
        break;

    case 2: // "CLEANING"
        statusDescription = "CLEANING";
        ledsControl();
        //squeegeeRight.write(0);
        //squeegeeLeft.write(0);

        if (!sensorFlag)
        {
            engineMotor.moveForward(255);

            if (maxDistance < frontDistance /*|| frontDistance == 0*/)
            {
                initTIME = currentTIME;
                sensorFlag = true;
            }
            // else
            // {
            //     //brushMotor.start();
            //     initTIME = currentTIME;
            // }
        }

        currentTIME = millis();
        if (currentTIME - initTIME >= intervalTIME)
        {
            initTIME = currentTIME;
            sensorFlag = false;

            if (maxDistance < frontDistance /*|| frontDistance == 0*/)
            {
                engineMotor.moveForward(0);
                cleaningLed.turnOn();
                delay(5000);
                status = 3;
            }
        }
        // else
        // {
        //     engineMotor.moveForward(127);
        // }
        break;

    case 3: // "SQUEEGEEING"
        statusDescription = "SQUEEGEEING";
        ledsControl();
        //squeegeeRight.write(180);
        //squeegeeLeft.write(180);

        if (!sensorFlag)
        {
            engineMotor.moveBackward(255);

            if (maxDistance < backDistance /*|| backDistance == 0*/)
            {
                initTIME = currentTIME;
                sensorFlag = true;
            }
            // else
            // {
            //     //brushMotor.start();
            //     initTIME = currentTIME;
            // }
        }

        currentTIME = millis();
        if (currentTIME - initTIME >= intervalTIME)
        {
            initTIME = currentTIME;
            sensorFlag = false;
            
            if (maxDistance < backDistance /*|| backDistance == 0*/)
            {
                engineMotor.moveBackward(0);
                squeegeeingLed.turnOn();
                delay(5000);
                status = 0;
            }
        }
        // else
        // {
        //     engineMotor.moveBackward(127);
        // }
        break;
        
    case 4: // "ERROR"
        statusDescription = "ERROR";
        ledsControl();
        valveMotor.stop();
        break;
    default:
        Operation::status = 0;
        break;
    }
    Serial.println("[status] | " + statusDescription + " |");
}

void Operation::turnOffMotors()
{
    engineMotor.moveBackward(0);
    //brushMotor.stop();
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
        squeegeeingLed.fade();
        break;
    case 4: // "ERROR"
        onoffLed.turnOn();
        cleaningLed.blink();
        squeegeeingLed.blink();
        break;
    default:
        Operation::status = 0;
        break;
    }
}
