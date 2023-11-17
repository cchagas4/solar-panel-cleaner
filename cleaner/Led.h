#ifndef ARDUINO_LED_H
#define ARDUINO_LED_H
#include <Arduino.h>

#define UP 0
#define DOWN 11
#define PWM_MIN 0
#define PWM_MAX 255
#define FADE_INCREMENT 5
#define FADE_INTERVAL 50

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

	byte led = 13;

	byte fadeDirection = UP;
	int fadeValue = 0;
	unsigned long previousFadeMillis;
	int fadeInterval = 50;

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
