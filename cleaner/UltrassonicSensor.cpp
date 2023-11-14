
#include "UltrassonicSensor.h"

UltrassonicSensor::UltrassonicSensor()
{
}

UltrassonicSensor::UltrassonicSensor(byte trigger, byte echo)
{
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
}

int UltrassonicSensor::getUltrasonicDistance()
{
    // Function to retreive the distance reading of the ultrasonic sensor
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

    // Uncomment this line to return value in IN instead of CM:
    // distance = distance * 0.3937008

    // Return the distance read from the sensor:
    return distance;
}