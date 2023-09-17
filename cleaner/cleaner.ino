// #include <Scheduler.h>
#include <Arduino.h>

#include "Led.h"
#include "Operation.h"
#include "Power.h"
// #include "Motor.h"

#define INDICATIVE_RED 13
#define INDICATIVE_BLUE 12
#define INDICATIVE_GREEN 11
#define EMERGENCY_BUTTON 2
#define ON_OFF 8

#define FORWARD_ENGINE 10
#define BACKWARD_ENGINE 9

#define BRUSH_ENGINE 6
#define VALVE_ENGINE 3

#define FRONT_SENSOR A0
#define BACK_SENSOR A1

// float brushMotor;            // port 5

// Motor motionEngine(FORWARD_ENGINE, BACKWARD_ENGINE);

// Motor brush(BRUSH_ENGINE);
// Motor valve(VALVE_ENGINE);

Power power(ON_OFF, EMERGENCY_BUTTON);
Led red(INDICATIVE_RED);
Led blue(INDICATIVE_BLUE);
Led green(INDICATIVE_GREEN);
Operation operation(red, blue, green);

bool emergencyMode;
bool powerFake = true;

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BUTTON), emergencyTrigger, CHANGE);
}

void loop()
{
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
  // delay(1000);
}

void emergencyTrigger()
{
  emergencyMode = true;
  power.off();
  Serial.println("--- | EMERGENCY BUTTON WAS TRIGGERED |");
  Serial.println("--- | TURNED OFF OPERATIONS |");
}