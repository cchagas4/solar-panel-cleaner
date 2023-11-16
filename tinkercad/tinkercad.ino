#include <Servo.h>

#include <Servo.h>

#ifndef ARDUINO_LED_H
#define ARDUINO_LED_H


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

Led::Led(byte pin)
{
    led = pin;
    pinMode(pin, OUTPUT);
}

Led::Led()
{
    pinMode(led, OUTPUT);
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
    analogWrite(led, brightness);

    brightness = brightness + fadeAmount;

    if (brightness == 0 || brightness == 255)
    {
        fadeAmount = -fadeAmount;
    }

    delay(300);
}

void Led::blink()
{
    digitalWrite(led, HIGH);
    delay(ledOnTime);
    digitalWrite(led, LOW);
    delay(ledOffTime);
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

	bool hasSpeedControl;

public:
	Motor();
	Motor(byte forwardMotorPin);
	Motor(byte forwardMotorPin, byte backwardMotorPin);

	void moveForward(int speed);
	void moveBackward(int speed);
	void stop();
	void start();
	// void startSmooth(bool isForward);
};
#endif

Motor::Motor()
{
}

Motor::Motor(byte actuator)
{
    hasSpeedControl = false;
    this->actuator = actuator;
    pinMode(actuator, OUTPUT);
}

Motor::Motor(byte forwardMotorPin, byte backwardMotorPin)
{
    this->forwardMotorPin = forwardMotorPin;
    this->backwardMotorPin = forwardMotorPin;

    pinMode(forwardMotorPin, OUTPUT);
    pinMode(backwardMotorPin, OUTPUT);

    hasSpeedControl = true;
}

void Motor::moveForward(int speed)
{
    analogWrite(forwardMotorPin, speed);
}
void Motor::moveBackward(int speed)
{
    analogWrite(backwardMotorPin, speed);
}

void Motor::stop()
{
    if (hasSpeedControl)
    {
        analogWrite(forwardMotorPin, MIN_SPEED);
        analogWrite(backwardMotorPin, MIN_SPEED);
    }
    else
    {
        digitalWrite(actuator, LOW);
    }
}

void Motor::start()
{
    if (hasSpeedControl)
    {
        analogWrite(forwardMotorPin, MAX_SPEED);
        analogWrite(backwardMotorPin, MAX_SPEED);
    }
    else
    {
        digitalWrite(actuator, LOW);
    }
}


#ifndef ARDUINO_SERVOMOTOR_H
#define ARDUINO_SERVOMOTOR_H

class ServoMotor
{
private:
	Servo servoMotor;
public:
	ServoMotor();
	ServoMotor(byte servoPin);

	void write(int degree);
};
#endif

ServoMotor::ServoMotor()
{

}

ServoMotor::ServoMotor(byte servoPin)
{
    servoMotor.attach(servoPin);
}

void ServoMotor::write(int degree){
    servoMotor.write(degree);
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

#ifndef ARDUINO_OPERATION_H
#define ARDUINO_OPERATION_H

class Operation
{
private:
	Led onoffLed;
	Led cleaningLed;
	Led squeegeeingLed;
	Motor engineMotor;
	Motor brushMotor;
	Motor valveMotor;
	ServoMotor squeegeeRight;
	ServoMotor squeegeeLeft;
	String statusDescription;
	UltrassonicSensor frontSensor;
	UltrassonicSensor backSensor;

	int maxDistance = 50;
	int frontDistance = 0;
	int backDistance = 0;
	void distancePrint(int distance);

public:
	Operation();
	void configLeds(Led onoff, Led cleaning, Led squeegeeing);
	void configMotors(Motor engine, Motor brush, Motor valve);
	void configServoMotors(ServoMotor right, ServoMotor left);
	void configUltrassonicSensors(UltrassonicSensor front, UltrassonicSensor back);
	inline static int status = 0;
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

void Operation::configMotors(Motor engine, Motor brush, Motor valve)
{
    engineMotor = engine;
    brushMotor = brush;
    valveMotor = valve;
}

void Operation::configServoMotors(ServoMotor right, ServoMotor left)
{
    squeegeeRight = right;
    squeegeeLeft = left;
}

void Operation::configUltrassonicSensors(UltrassonicSensor front, UltrassonicSensor back)
{
    frontSensor = front;
    backSensor = back;
}

void Operation::control()
{
    switch (Operation::status)
    {

    case 0: // "OFF"
        statusDescription = "OFF";
        onoffLed.turnOff();
        cleaningLed.turnOff();
        squeegeeingLed.turnOff();
        // TODO turn off motors???
        break;
    case 1: // "ON" TODO
        statusDescription = "ON";
        onoffLed.turnOn();
        cleaningLed.turnOff();
        squeegeeingLed.turnOff();
        // start cleaning
        status = 2;
        break;
    case 2: // "CLEANING"
        statusDescription = "CLEANING";
        cleaningLed.turnOn();
        squeegeeingLed.turnOff();
        squeegeeRight.write(0);
        squeegeeLeft.write(0);

        frontDistance = frontSensor.getUltrasonicDistance();
        distancePrint(frontDistance);
        if (maxDistance < frontDistance || frontDistance != 0) // TODO refactor
        {
            engineMotor.moveForward(255);
            brushMotor.start();
            valveMotor.start();
        }
        else
        {
            status = 3;
        }

        break;
    case 3: // "SQUEEGEEING"
        statusDescription = "SQUEEGEEING";
        cleaningLed.turnOff();
        squeegeeingLed.turnOn();
        squeegeeRight.write(180);
        squeegeeLeft.write(180);

        backDistance = backSensor.getUltrasonicDistance();
        distancePrint(backDistance);
        if (maxDistance < backDistance || backDistance != 0) // TODO refactor
        {
            engineMotor.moveBackward(255);
            brushMotor.stop();
            valveMotor.stop();
        }
        else
        {
            status = 0;
        }
        break;
    case 4: // "ERROR"
        statusDescription = "ERROR";
        cleaningLed.turnOn();
        squeegeeingLed.turnOn();
        engineMotor.stop();
        brushMotor.stop();
        valveMotor.stop();
        break;
    default:
        Operation::status = 4;
        break;
    }
    Serial.println("STATUS ---> [" + statusDescription + "]");
}

void Operation::distancePrint(int distance)
{
    Serial.print("OBSTACLE ---> [");
    Serial.print(distance);
    Serial.println("]");
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

public:
	Power(byte power, byte emergency);

	void on();
	void off();
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

bool Power::isOn()
{
    isPowerOn = digitalRead(onOff);
    // Debug
    String currentStatus = isPowerOn == HIGH ? "ON" : "OFF";
    Serial.println("POWER ---> [" + currentStatus + "]");
    if (isPowerOn == LOW)
    {
        Serial.println("Turning Off ---> [" + currentStatus + "]");
        off();
    } else if(lastStatus != currentStatus){
        on();
    }

    lastStatus = currentStatus;
    return isPowerOn;
}

void Power::setPinsToLow()
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
            digitalWrite(pin, LOW);
        }
    }
}

#define INDICATIVE_RED 13
#define INDICATIVE_BLUE 12
#define INDICATIVE_GREEN 11
#define EMERGENCY_BUTTON 2
#define ON_OFF 8

#define FORWARD_ENGINE 10
#define BACKWARD_ENGINE 9

// #define BRUSH_ENGINE 6 // TODO?
#define VALVE_ENGINE 3

// TODO 4 pins to ultrassonic sensors
#define FRONT_TRIGGER 7
#define FRONT_ECHO 6
#define BACK_TRIGGER 5
#define BACK_ECHO 4

#define SQUEEGEE_RIGHT A4
#define SQUEEGEE_LEFT A5

// #define FRONT_SENSOR A0 TODO
// #define BACK_SENSOR A1 TODO

// float brushMotor;            // port 5

Motor motionEngine(FORWARD_ENGINE, BACKWARD_ENGINE);

// Motor brush(BRUSH_ENGINE);
Motor brush; // TODO wich type of motor will be used on brush???
Motor valve(VALVE_ENGINE);

ServoMotor squeegeeRight(SQUEEGEE_RIGHT);
ServoMotor squeegeeLeft(SQUEEGEE_LEFT);

Power power(ON_OFF, EMERGENCY_BUTTON);
Led red(INDICATIVE_RED);
Led blue(INDICATIVE_BLUE);
Led green(INDICATIVE_GREEN);
Operation operation;

UltrassonicSensor front(FRONT_TRIGGER, FRONT_ECHO); // TODO FIX pins
UltrassonicSensor back(BACK_TRIGGER, BACK_ECHO);    // TODO FIX pins

bool emergencyMode;
bool powerFake = true; // TODO WTF???

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BUTTON), emergencyTrigger, CHANGE);
  initialConfiguration();
}

void loop()
{
  // Checking if emergency mode are trigged
  Serial.println("--- | emergency mode --> " + String(emergencyMode));
  if (!emergencyMode)
  {
    Serial.println("--- | Checking power on button status |");
    if (power.isOn())
    {
      Serial.println("--- | Control |");
      operation.control();
    }
  }
  delay(5000); // TODO avoid delays
}

void initialConfiguration()
{
  operation.configLeds(red, blue, green);
  operation.configMotors(motionEngine, brush, valve);
  operation.configServoMotors(squeegeeRight, squeegeeLeft);
  operation.configUltrassonicSensors(front, back);
}

void emergencyTrigger()
{
  emergencyMode = true;
  power.off();
  Serial.println("--- | EMERGENCY BUTTON WAS TRIGGERED |");
  Serial.println("--- | TURNED OFF OPERATIONS |");
}