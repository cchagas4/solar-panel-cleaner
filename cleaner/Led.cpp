#include "Led.h"

Led::Led()
{
}

void Led::config()
{
    pinMode(led, OUTPUT);
}

void Led::changeIndicativeLed(byte led)
{
    this->led = led;
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
    digitalWrite(led, HIGH);
    delay(ledOnTime);
    digitalWrite(led, LOW);
    delay(ledOffTime);
}

void Led::blinkCustom(int ledOnTime, int ledOffTime)
{
    Led::ledOnTime = ledOnTime;
    Led::ledOffTime = ledOffTime;
    blink();
}