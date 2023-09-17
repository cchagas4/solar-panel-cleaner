#ifndef ARDUINO_LED_H
#define ARDUINO_LED_H
#include <Arduino.h>

class Led
{
private:
	inline static int brightness = 0;
	inline static int fadeAmount = 5;
	inline static int ledOnTime = 300;
	inline static int ledOffTime = 200;

	byte led = 13; // Default value should be subscrived

public:
	Led(byte pin);
	Led();

	void fade();
	void blink();
	void blinkCustom(int ledOnTime, int ledOffTime);
	void turnOn();
	void turnOff();
	void setPinLed(byte pin);
	int getPin();
};
#endif
