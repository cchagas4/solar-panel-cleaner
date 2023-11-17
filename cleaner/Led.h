#ifndef ARDUINO_LED_H
#define ARDUINO_LED_H
#include <Arduino.h>

#define UP 0
#define DOWN 1

class Led
{
private:
	inline static int brightness = 0;
	inline static int fadeAmount = 5;
	inline static int ledOnTime = 200;
	inline static int ledOffTime = 100;
	inline static int interval = 0;
	inline static int ledState = LOW;
	unsigned long initTime = 0;
	unsigned long currentTime;

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
