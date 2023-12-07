#include <Arduino.h>
#include "Led.h"
#include "Motor.h"
//#include "ServoMotor.h"
#include "UltrassonicSensor.h"
#include "Operation.h"
#include "Power.h"

#define INDICATIVE_RED 13
#define INDICATIVE_BLUE 3
#define INDICATIVE_GREEN 11

#define EMERGENCY_BUTTON 2
#define ON_OFF 8

#define FORWARD_ENGINE 10
#define BACKWARD_ENGINE 9
//#define BRUSH_ENGINE 3
#define VALVE_ENGINE A0

#define FRONT_TRIGGER 7
#define FRONT_ECHO 6
#define BACK_TRIGGER 5
#define BACK_ECHO 4

//#define SQUEEGEE_RIGHT A4
//#define SQUEEGEE_LEFT A5

Motor motionEngine(FORWARD_ENGINE, BACKWARD_ENGINE);

//Motor brush(BRUSH_ENGINE);
Motor valve(VALVE_ENGINE);

//ServoMotor squeegeeRight(SQUEEGEE_RIGHT);
//ServoMotor squeegeeLeft(SQUEEGEE_LEFT);

Power power(ON_OFF, EMERGENCY_BUTTON);
Led red(INDICATIVE_RED);
Led blue(INDICATIVE_BLUE);
Led green(INDICATIVE_GREEN);
Operation operation;

UltrassonicSensor front(FRONT_TRIGGER, FRONT_ECHO);
UltrassonicSensor back(BACK_TRIGGER, BACK_ECHO);

bool emergencyMode;

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BUTTON), emergencyTrigger, CHANGE);
  initialConfiguration();
}

void loop()
{
  // Checking if emergency mode are trigged
  while (!emergencyMode)
  {
    if (power.isOn())
    {
      //Serial.println("[main] | Calling Control |");
      operation.control();
    }
  }
  // delay(1000); // TODO avoid delays
}

void initialConfiguration()
{
  operation.configLeds(red, blue, green);
  operation.configMotors(motionEngine, valve);
  //operation.configServoMotors(squeegeeRight, squeegeeLeft);
  operation.configUltrassonicSensors(front, back);
}

void emergencyTrigger()
{
  emergencyMode = true;
  power.emergencyMode();
  Serial.println("[main] | EMERGENCY BUTTON WAS TRIGGERED |");
  Serial.println("[main] | TURNED OFF OPERATIONS |");
}