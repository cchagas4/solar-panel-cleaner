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
    this->onOff = onOff;
    this->emergency = emergency;
    Operation::status = 0;
}

void Power::on()
{
    isPowerOn = HIGH;
    Operation::status = 1;
}

void Power::off()
{
    isPowerOn = LOW;
    setPinsToLow();
    Operation::status = 0;
}

bool Power::isOn()
{
    isPowerOn = digitalRead(onOff);
    // Debug
    String currentStatus = isPowerOn == HIGH ? "ON" : "OFF";
    Serial.println("POWER ---> [" + currentStatus + "]");
    if (isPowerOn == LOW)
    {
        off();
    } else if(lastStatus != currentStatus){
        on();
    }

    lastStatus = currentStatus;
    return isPowerOn;
}

void Power::setPinsToLow()
{
    for (int pin : pins)
    {
        if (pin == onOff || pin == emergency)
        {
            pinMode(pin, INPUT_PULLUP);
        }
        else
        {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
    }
}
