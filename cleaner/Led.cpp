#include "Led.h"

const int minPWM = 0;
const int maxPWM = 255;
byte fadeDirection = UP;
int fadeValue = 0;
byte fadeIncrement = 5;
unsigned long previousFadeMillis;
int fadeInterval = 50;

Led::Led(byte pin)
{
    led = pin;
    pinMode(pin, OUTPUT);
}

Led::Led()
{
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

    unsigned long thisMillis = millis();

    if (thisMillis - previousFadeMillis >= fadeInterval)
    {
        if (fadeDirection == UP)
        {
            fadeValue = fadeValue + fadeIncrement;
            if (fadeValue >= maxPWM)
            {
                fadeValue = maxPWM;
                fadeDirection = DOWN;
            }
        }
        else
        {
            fadeValue = fadeValue - fadeIncrement;
            if (fadeValue <= minPWM)
            {
                fadeValue = minPWM;
                fadeDirection = UP;
            }
        }
        analogWrite(led, fadeValue);
        previousFadeMillis = thisMillis;
    }
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