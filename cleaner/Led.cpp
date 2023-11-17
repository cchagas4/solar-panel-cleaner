#include "Led.h"

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

    if (thisMillis - previousFadeMillis >= FADE_INTERVAL)
    {
        if (fadeDirection == UP)
        {
            fadeValue = fadeValue + FADE_INCREMENT;
            if (fadeValue >= PWM_MAX)
            {
                fadeValue = PWM_MAX;
                fadeDirection = DOWN;
            }
        }
        else
        {
            fadeValue = fadeValue - FADE_INCREMENT;
            if (fadeValue <= PWM_MIN)
            {
                fadeValue = PWM_MIN;
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