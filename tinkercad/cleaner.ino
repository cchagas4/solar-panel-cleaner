/////////////
#include <Arduino.h>

#ifndef ARDUINO_LED_H
#define ARDUINO_LED_H

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
	inline static int ledOnTime = 750;
	inline static int ledOffTime = 750;
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

Led::Led(byte pin)
{
    led = pin;
    pinMode(pin, OUTPUT);
}

Led::Led()
{
}

void Led::turnOn()
{
    digitalWrite(led, HIGH);
}

void Led::turnOff()
{
    digitalWrite(led, LOW);
}

void Led::setPinLed(byte pin)
{
    led = pin;
    pinMode(pin, OUTPUT);
}

void Led::fade()
{

    unsigned long thisMillis = millis();

    if (thisMillis - previousFadeMillis >= FADE_INTERVAL)
    {
        if (fadeDirection == UP)
        {
            fadeValue = fadeValue + FADE_INCREMENT;
            if (fadeValue >= PWM_MAX)
            {
                fadeValue = PWM_MAX;
                fadeDirection = DOWN;
            }
        }
        else
        {
            fadeValue = fadeValue - FADE_INCREMENT;
            if (fadeValue <= PWM_MIN)
            {
                fadeValue = PWM_MIN;
                fadeDirection = UP;
            }
        }
        analogWrite(led, fadeValue);
        previousFadeMillis = thisMillis;
    }
}

void Led::blink()
{
    currentTime = millis();

    if (ledState)
    {
        interval = ledOnTime;
    }
    else
    {
        interval = ledOffTime;
    }

    if (currentTime - initTime >= interval)
    {
        initTime = currentTime;
        ledState = !ledState;
        digitalWrite(led, ledState);
    }
}

void Led::blinkCustom(int ledOnTime, int ledOffTime)
{
    Led::ledOnTime = ledOnTime;
    Led::ledOffTime = ledOffTime;
    blink();
}

int Led::getPin()
{
    return this->led;
}


#ifndef ARDUINO_MOTOR_H
#define ARDUINO_MOTOR_H
class Motor
{
private:
	const static int MIN_SPEED = 0;
	const static int MAX_SPEED = 255;

	byte forwardMotorPin;
	byte backwardMotorPin;
	byte actuator;

	bool hasDirection;

public:
	Motor();
	Motor(byte actuator);
	Motor(byte forwardMotorPin, byte backwardMotorPin);

	void moveForward(int speed);
	void moveBackward(int speed);
	void stop();
	void start();
};
#endif

#include "Motor.h"

Motor::Motor()
{
}

Motor::Motor(byte actuator)
{
    this->hasDirection = false;
    this->actuator = actuator;
    pinMode(actuator, OUTPUT);
}

Motor::Motor(byte forwardMotorPin, byte backwardMotorPin)
{
    this->forwardMotorPin = forwardMotorPin;
    this->backwardMotorPin = backwardMotorPin;

    pinMode(forwardMotorPin, OUTPUT);
    pinMode(backwardMotorPin, OUTPUT);

    this->hasDirection = true;
}

void Motor::moveForward(int speed)
{
    analogWrite(backwardMotorPin, MIN_SPEED);
    analogWrite(forwardMotorPin, speed);
}

void Motor::moveBackward(int speed)
{
    analogWrite(forwardMotorPin, MIN_SPEED);
    analogWrite(backwardMotorPin, speed);
}

void Motor::stop()
{
    digitalWrite(actuator, LOW);
}

void Motor::start()
{
    digitalWrite(actuator, HIGH);
}


#ifndef ARDUINO_POWER_H
#define ARDUINO_POWER_H

class Power
{
private:
	byte pins[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
	int isPowerOn;
	byte onOff;
	byte emergency;
	String lastStatus;
	void setPinsToLow();
	void setPinsToEmergencyMode();
	void configAllPins();

public:
	Power(byte power, byte emergency);

	void on();
	void off();
	void emergencyMode();
	bool isOn();
};
#endif

/* STATUS
 * 0 -- ON
 * 1 -- OFF
 * 2 -- CLEANING
 * 3 -- STARTING
 * 4 -- ERROR
 */

Power::Power(byte onOff, byte emergency)
{
    this->onOff = onOff;
    this->emergency = emergency;
    Operation::status = 0;
}

void Power::on()
{
    isPowerOn = HIGH;
    Operation::status = 1;
}

void Power::off()
{
    isPowerOn = LOW;
    setPinsToLow();
    Operation::status = 0;
}

void Power::emergencyMode()
{
    isPowerOn = LOW;
    setPinsToEmergencyMode();
    Operation::status = 4;
}

bool Power::isOn()
{
    isPowerOn = digitalRead(onOff);

    String currentStatus = isPowerOn == HIGH ? "ON" : "OFF";
    // Serial.println("[power] | [" + currentStatus + "]");
    if (isPowerOn == LOW)
    {
        Serial.println("[power] | Turning Off ---> [" + currentStatus + "]");
        off();
    }
    else if (lastStatus != currentStatus)
    {
        on();
    }

    lastStatus = currentStatus;
    return isPowerOn;
}

void Power::setPinsToLow()
{
    for (int pin : pins)
    {
        digitalWrite(pin, LOW);
    }
}

void Power::setPinsToEmergencyMode()
{
    configAllPins();
    setPinsToLow();
}

void Power::configAllPins()
{
    for (int pin : pins)
    {
        if (pin == onOff || pin == emergency)
        {
            pinMode(pin, INPUT_PULLUP);
        }
        else
        {
            pinMode(pin, OUTPUT);
        }
    }
}


#ifndef ARDUINO_ULTRASSONIC_SENSOR_H
#define ARDUINO_ULTRASSONIC_SENSOR_H

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


UltrassonicSensor::UltrassonicSensor()
{
}

UltrassonicSensor::UltrassonicSensor(byte trigger, byte echo)
{
    this->trigger = trigger;
    this->echo = echo;
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
}

int UltrassonicSensor::getUltrasonicDistance()
{
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
    duration = pulseIn(echo, HIGH, 10000UL); // TODO aprox 1m
    // Calculate the distance:
    distance = duration * 0.034 / 2;

    // Return the distance read from the sensor:
    return distance;
}

#ifndef ARDUINO_OPERATION_H
#define ARDUINO_OPERATION_H

class Operation
{
private:
	Led onoffLed;
	Led cleaningLed;
	Led squeegeeingLed;
	Motor engineMotor;
	//Motor brushMotor;
	Motor valveMotor;
	//ServoMotor squeegeeRight;
	//ServoMotor squeegeeLeft;
	String statusDescription;
	UltrassonicSensor frontSensor;
	UltrassonicSensor backSensor;

	int maxDistance = 100;
	int frontDistance = 0;
	int backDistance = 0;
	int intervalTIME = 1000;
	unsigned long initTIME = 0;
	unsigned long currentTIME;
	void distancePrint(int distance);

	void turnOffMotors();
	void ledsControl();

public:
	Operation();
	void configLeds(Led onoff, Led cleaning, Led squeegeeing);
	void configMotors(Motor engine, Motor valve);
	//void configServoMotors(ServoMotor right, ServoMotor left);
	void configUltrassonicSensors(UltrassonicSensor front, UltrassonicSensor back);
	inline static int status = 0;
	bool sensorFlag = false;
	void control();
};
#endif

Operation::Operation()
{
}

void Operation::configLeds(Led onoff, Led cleaning, Led squeegeeing)
{
    onoffLed = onoff;
    cleaningLed = cleaning;
    squeegeeingLed = squeegeeing;
}

void Operation::configMotors(Motor engine, Motor valve)
{
    engineMotor = engine;
    //brushMotor = brush;
    valveMotor = valve;
}

// void Operation::configServoMotors(ServoMotor right, ServoMotor left)
// {
//     squeegeeRight = right;
//     squeegeeLeft = left;
// }

void Operation::configUltrassonicSensors(UltrassonicSensor front, UltrassonicSensor back)
{
    frontSensor = front;
    backSensor = back;
}

void Operation::control()
{
    frontDistance = frontSensor.getUltrasonicDistance();
    backDistance = backSensor.getUltrasonicDistance();
    Serial.println("[CONTROL] | FRONT DISTANCE --> " + String(frontDistance));
    Serial.println("[CONTROL] | BACK DISTANCE --> " + String(backDistance));

    // if (frontDistance == 0 && backDistance == 0)
    // {
    //     status = 4;
    // }

    switch (Operation::status)
    {
    case 0: // "OFF"
        statusDescription = "OFF";
        ledsControl();
        valveMotor.stop();
        break;

    case 1: // "ON" TODO
        statusDescription = "ON";
        ledsControl();
        valveMotor.start();
        delay(10000); // Waiting for the water to start falling
        status = 2;
        break;

    case 2: // "CLEANING"
        statusDescription = "CLEANING";
        ledsControl();
        //squeegeeRight.write(0);
        //squeegeeLeft.write(0);

        if (!sensorFlag)
        {
            if (maxDistance < frontDistance || frontDistance == 0)
            {
                sensorFlag = true;
            }
            else
            {
                engineMotor.moveForward(255);
                //brushMotor.start();
                initTIME = currentTIME;
            }
        }

        currentTIME = millis();
        if (currentTIME - initTIME >= intervalTIME)
        {
            initTIME = currentTIME;
            sensorFlag = false;

            if (maxDistance < frontDistance || frontDistance == 0)
            {
                engineMotor.moveForward(0);
                cleaningLed.turnOn();
                delay(5000);
                status = 3;
            }
        }
        else
        {
            engineMotor.moveForward(127);
        }
        break;

    case 3: // "SQUEEGEEING"
        statusDescription = "SQUEEGEEING";
        ledsControl();
        //squeegeeRight.write(180);
        //squeegeeLeft.write(180);

        if (!sensorFlag)
        {
            if (maxDistance < backDistance || backDistance == 0)
            {
                sensorFlag = true;
            }
            else
            {
                engineMotor.moveBackward(255);
                //brushMotor.start();
                initTIME = currentTIME;
            }
        }

        currentTIME = millis();
        if (currentTIME - initTIME >= intervalTIME)
        {
            initTIME = currentTIME;
            sensorFlag = false;
            
            if (maxDistance < backDistance || backDistance == 0)
            {
                engineMotor.moveBackward(0);
                squeegeeingLed.turnOn();
                delay(5000);
                status = 0;
            }
        }
        else
        {
            engineMotor.moveBackward(127);
        }
        break;
        
    case 4: // "ERROR"
        statusDescription = "ERROR";
        ledsControl();
        valveMotor.stop();
        break;
    default:
        Operation::status = 0;
        break;
    }
    Serial.println("[status] | " + statusDescription + " |");
}

void Operation::turnOffMotors()
{
    engineMotor.moveBackward(0);
    //brushMotor.stop();
    valveMotor.stop();
}

void Operation::ledsControl()
{
    switch (Operation::status)
    {
    case 0: // "OFF"
        onoffLed.blink();
        cleaningLed.turnOff();
        squeegeeingLed.turnOff();
        break;
    case 1: // "ON" TODO
        onoffLed.turnOn();
        cleaningLed.turnOff();
        squeegeeingLed.turnOff();
        break;
    case 2: // "CLEANING"
        cleaningLed.fade();
        squeegeeingLed.turnOff();
        break;
    case 3: // "SQUEEGEEING"
        cleaningLed.turnOff();
        squeegeeingLed.fade();
        break;
    case 4: // "ERROR"
        onoffLed.turnOn();
        cleaningLed.blink();
        squeegeeingLed.blink();
        break;
    default:
        Operation::status = 0;
        break;
    }
}

#define INDICATIVE_RED 13
#define INDICATIVE_BLUE 3
#define INDICATIVE_GREEN 11

#define EMERGENCY_BUTTON 2
#define ON_OFF 8

#define FORWARD_ENGINE 10
#define BACKWARD_ENGINE 9
//#define BRUSH_ENGINE 3
#define VALVE_ENGINE A0

#define FRONT_TRIGGER 7
#define FRONT_ECHO 6
#define BACK_TRIGGER 5
#define BACK_ECHO 4

//#define SQUEEGEE_RIGHT A4
//#define SQUEEGEE_LEFT A5

Motor motionEngine(FORWARD_ENGINE, BACKWARD_ENGINE);

//Motor brush(BRUSH_ENGINE);
Motor valve(VALVE_ENGINE);

//ServoMotor squeegeeRight(SQUEEGEE_RIGHT);
//ServoMotor squeegeeLeft(SQUEEGEE_LEFT);

Power power(ON_OFF, EMERGENCY_BUTTON);
Led red(INDICATIVE_RED);
Led blue(INDICATIVE_BLUE);
Led green(INDICATIVE_GREEN);
Operation operation;

UltrassonicSensor front(FRONT_TRIGGER, FRONT_ECHO);
UltrassonicSensor back(BACK_TRIGGER, BACK_ECHO);

bool emergencyMode;

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BUTTON), emergencyTrigger, CHANGE);
  initialConfiguration();
}

void loop()
{
  // Checking if emergency mode are trigged
  while (!emergencyMode)
  {
    if (power.isOn())
    {
      //Serial.println("[main] | Calling Control |");
      operation.control();
    }
  }
  // delay(1000); // TODO avoid delays
}

void initialConfiguration()
{
  operation.configLeds(red, blue, green);
  operation.configMotors(motionEngine, valve);
  //operation.configServoMotors(squeegeeRight, squeegeeLeft);
  operation.configUltrassonicSensors(front, back);
}

void emergencyTrigger()
{
  emergencyMode = true;
  power.emergencyMode();
  Serial.println("[main] | EMERGENCY BUTTON WAS TRIGGERED |");
  Serial.println("[main] | TURNED OFF OPERATIONS |");
}
