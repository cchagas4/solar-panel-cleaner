#include "Led.h"

Led::Led(byte pin)
{
    led = pin;
    pinMode(pin, OUTPUT);
}

Led::Led()
{
    pinMode(led, OUTPUT);
}

void Led::turnOn()
{
    digitalWrite(led, HIGH);
}

void Led::turnOff()
{
    digitalWrite(led, LOW);
}

void Led::setPinLed(byte pin)
{
    led = pin;
    pinMode(pin, OUTPUT);
}

void Led::fade()
{
    analogWrite(led, brightness);

    brightness = brightness + fadeAmount;

    if (brightness == 0 || brightness == 255)
    {
        fadeAmount = -fadeAmount;
    }

    delay(300);
}

void Led::blink()
{
	currentTime = millis();

    if (ledState)
    {
        interval = ledOnTime;
    }
    else
    {
        interval = ledOffTime;
    }

    if (currentTime - initTime >= interval) 
    {
        initTime = currentTime;
        ledState = !ledState;
        digitalWrite(led, ledState);
    }
}

void Led::blinkCustom(int ledOnTime, int ledOffTime)
{
    Led::ledOnTime = ledOnTime;
    Led::ledOffTime = ledOffTime;
    blink();
}

int Led::getPin()
{
    return this->led;
}