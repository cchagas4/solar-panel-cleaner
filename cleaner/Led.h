
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
	inline static byte led = 13;
	int status;

public:
	Led();

	static void config();
	static void fade();
	static void blink();
	static void blinkCustom(int ledOnTime, int ledOffTime);
	void changeIndicativeLed(byte led);
};
#endif
