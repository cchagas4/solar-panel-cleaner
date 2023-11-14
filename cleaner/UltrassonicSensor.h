#ifndef ARDUINO_ULTRASSONIC_SENSOR_H
#define ARDUINO_ULTRASSONIC_SENSOR_H
#include <Arduino.h>

class UltrassonicSensor
{
private:
    byte trigger;
    byte echo;

public:
    UltrassonicSensor();
    UltrassonicSensor(byte trigger, byte echo);
    int getUltrasonicDistance();
};

#endif