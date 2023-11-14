// #include <Scheduler.h>
#include <Arduino.h>

#include "Led.h"
#include "Motor.h"
#include "Operation.h"
#include "Power.h"
#include "UltrassonicSensor.h"

#define INDICATIVE_RED 13
#define INDICATIVE_BLUE 12
#define INDICATIVE_GREEN 11
#define EMERGENCY_BUTTON 2
#define ON_OFF 8

#define FORWARD_ENGINE 10
#define BACKWARD_ENGINE 9

#define BRUSH_ENGINE 6
#define VALVE_ENGINE 3

// TODO 4 pins to ultrassonic sensors
#define FRONT_TRIGGER A0
#define FRONT_ECHO A1
#define BACK_TRIGGER A2
#define BACK_ECHO A3

// #define FRONT_SENSOR A0 TODO
// #define BACK_SENSOR A1 TODO

// float brushMotor;            // port 5

Motor motionEngine(FORWARD_ENGINE, BACKWARD_ENGINE);

Motor brush(BRUSH_ENGINE);
Motor valve(VALVE_ENGINE);

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
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BUTTON), emergencyTrigger, CHANGE);
  initialConfiguration();
}

void loop()
{
  // Checking if emergency mode are trigged
  Serial.println("--- | emergency mode --> " + String(emergencyMode));
  if (!emergencyMode)
  {
    Serial.println("--- | Checking power on button status |");
    if (power.isOn())
    {
      Serial.println("--- | Control |");
      operation.control();
    }
  }
  // delay(10000); // TODO avoid delays
}

void initialConfiguration()
{
  operation.configLeds(red, blue, green);
  operation.configMotors(motionEngine, brush, valve);
  operation.configUltrassonicSensors(front, back);
}

void emergencyTrigger()
{
  emergencyMode = true;
  power.off();
  Serial.println("--- | EMERGENCY BUTTON WAS TRIGGERED |");
  Serial.println("--- | TURNED OFF OPERATIONS |");
}