
#include "UltrassonicSensor.h"

UltrassonicSensor::UltrassonicSensor()
{
}

UltrassonicSensor::UltrassonicSensor(byte trigger, byte echo)
{
    this->trigger = trigger;
    this->echo = echo;
}

int UltrassonicSensor::getUltrasonicDistance()
{
    // TODO dicover why not working outside here (pins configurations)
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);

    long duration;
    int distance;

    // Assure the trigger pin is LOW:
    digitalWrite(trigger, LOW);
    // Brief pause:
    delayMicroseconds(5);

    // Trigger the sensor by setting the trigger to HIGH:
    digitalWrite(trigger, HIGH);
    // Wait a moment before turning off the trigger:
    delayMicroseconds(10);
    // Turn off the trigger:
    digitalWrite(trigger, LOW);

    // Read the echo pin:
    duration = pulseIn(echo, HIGH);
    // Calculate the distance:
    distance = duration * 0.034 / 2;

    // Return the distance read from the sensor:
    return distance;
}