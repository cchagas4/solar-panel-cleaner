#ifndef ARDUINO_OPERATION_H
#define ARDUINO_OPERATION_H
#include <Arduino.h>
#include "Led.h"

class Operation
{
private:
	Led onoffLed;
	Led cleaningLed;
	Led squeegeeingLed;
	String statusDescription;

public:
	Operation(Led onoff,
			  Led cleaning,
			  Led squeegeeing);

	inline static int status = 0;
	void control();
};
#endif