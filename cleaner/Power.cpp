// #include <list>

#include "Power.h"

/* STATUS
 * 0 -- ON
 * 1 -- OFF
 * 2 -- CLEANING
 * 3 -- STARTING
 * 4 -- ERROR
 */

Power::Power(byte onOff, byte emergency)
{
    pinMode(onOff, INPUT);
    pinMode(emergency, INPUT);
    powerOff();

    Operation::status = 3; // STARTING
}

void Power::powerOn()
{
    isOn = true;
}

void Power::powerOff()
{
    isOn = false;
    for (int pin : pins)
    {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}