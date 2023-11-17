#include <Arduino.h>

#include "Led.h"
#include "Motor.h"
#include "ServoMotor.h"
#include "UltrassonicSensor.h"
#include "Operation.h"
#include "Power.h"

#define INDICATIVE_RED 13
#define INDICATIVE_BLUE 12
#define INDICATIVE_GREEN 11
#define EMERGENCY_BUTTON 2
#define ON_OFF 8

#define FORWARD_ENGINE 10
#define BACKWARD_ENGINE 9

// #define BRUSH_ENGINE 6 // TODO?
#define VALVE_ENGINE 3

// TODO 4 pins to ultrassonic sensors
#define FRONT_TRIGGER 7
#define FRONT_ECHO 6
#define BACK_TRIGGER 5
#define BACK_ECHO 4

#define SQUEEGEE_RIGHT A4
#define SQUEEGEE_LEFT A5

// #define FRONT_SENSOR A0 TODO
// #define BACK_SENSOR A1 TODO

// float brushMotor;            // port 5

Motor motionEngine(FORWARD_ENGINE, BACKWARD_ENGINE);

// Motor brush(BRUSH_ENGINE);
Motor brush; // TODO wich type of motor will be used on brush???
Motor valve(VALVE_ENGINE);

ServoMotor squeegeeRight(SQUEEGEE_RIGHT);
ServoMotor squeegeeLeft(SQUEEGEE_LEFT);

Power power(ON_OFF, EMERGENCY_BUTTON);
Led red(INDICATIVE_RED);
Led blue(INDICATIVE_BLUE);
Led green(INDICATIVE_GREEN);
Operation operation;

UltrassonicSensor front(FRONT_TRIGGER, FRONT_ECHO); // TODO FIX pins
UltrassonicSensor back(BACK_TRIGGER, BACK_ECHO);    // TODO FIX pins

bool emergencyMode;
bool powerFake = true; // TODO WTF???

void setup()
{
  // blue.turnOn();
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BUTTON), emergencyTrigger, CHANGE);
  initialConfiguration();
  blue.blink();

  if (power.isOn())
  {
    red.turnOn();
  }
}

void loop()
{
  // Checking if emergency mode are trigged
  if (!emergencyMode)
  {
    if (power.isOn())
    {
      Serial.println("[main] | Calling Control |"); // TODO uncomment
      operation.control();                          // TODO uncomment
    }
  }
  else
  {
    Serial.println("[main] | EMERGENCY MODE |");
  }
  // delay(2000); // TODO avoid delays
}

void initialConfiguration()
{
  operation.configLeds(red, green, blue);
  operation.configMotors(motionEngine, brush, valve);
  operation.configServoMotors(squeegeeRight, squeegeeLeft);
  operation.configUltrassonicSensors(front, back);
}

void emergencyTrigger()
{
  emergencyMode = true;
  power.off();
  Serial.println("[main] | EMERGENCY BUTTON WAS TRIGGERED |");
  Serial.println("[main] | TURNED OFF OPERATIONS |");
}