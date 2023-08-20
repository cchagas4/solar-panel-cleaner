
#ifndef ARDUINO_OPERATION_H
#define ARDUINO_OPERATION_H
#include <Arduino.h>
#include "Led.h"

class Operation
{
private:
public:
	Operation();

	inline static int status = 0;
	static void control();
};
#endif