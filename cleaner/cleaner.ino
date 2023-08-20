#include <Scheduler.h>

#include "Led.h"
#include "Operation.h"
#include "Power.h"
#include "Motor.h"

#define INDICATIVE_LED 13
#define EMERGENCY_BUTTON 12
#define ON_OFF 8

#define FORWARD_ENGINE 11
#define BACKWARD_ENGINE 10

#define BRUSH_ENGINE 6
#define VALVE_ENGINE 3

#define FRONT_SENSOR A0
#define BACK_SENSOR A1

// float brushMotor;            // port 5

Motor motionEngine(FORWARD_ENGINE, BACKWARD_ENGINE);

Motor brush(BRUSH_ENGINE);
Motor valve(VALVE_ENGINE);

Power power(ON_OFF, EMERGENCY_BUTTON);

void setup()
{
  Led::config();
  Scheduler.startLoop(secundaryLoop);
}

void loop()
{
  // put your main code here, to run repeatedly:
}

void secundaryLoop()
{
  Operation::control();
}