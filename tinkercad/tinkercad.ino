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
    String statusDescription;

public:
    Operation();
    void configLeds(Led onoff, Led cleaning, Led squeegeeing);
    void configMotors(Motor engine, Motor brush, Motor valve);
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

void Operation::control()
{
    switch (Operation::status)
    {

    case 0: // "OFF"
        statusDescription = "OFF";
        onoffLed.turnOff();
        cleaningLed.turnOff();
        squeegeeingLed.turnOff();
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
        engineMotor.moveForward(255);
        brushMotor.start();
        valveMotor.start();
        break;
    case 3: // "SQUEEGEEING"
        statusDescription = "SQUEEGEEING";
        cleaningLed.turnOff();
        squeegeeingLed.turnOn();
        engineMotor.moveBackward(255);
        brushMotor.stop();
        valveMotor.stop();
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

#ifndef ARDUINO_POWER_H
#define ARDUINO_POWER_H

class Power
{
private:
    byte pins[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    int isPowerOn;
    byte onOff;
    byte emergency;
    void setPinsToLow();
    String lastStatus;

public:
    Power(byte power, byte emergency);

    void on();
    void off();
    bool isOn();
};
#endif

// #include <list>

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

// -----------------------------
// --------- MAIN --------------

#define INDICATIVE_RED 13
#define INDICATIVE_BLUE 12
#define INDICATIVE_GREEN 11
#define EMERGENCY_BUTTON 2
#define ON_OFF 8

#define FORWARD_ENGINE 10
#define BACKWARD_ENGINE 9

#define BRUSH_ENGINE 6
#define VALVE_ENGINE 3

#define FRONT_SENSOR A0
#define BACK_SENSOR A1

// float brushMotor;            // port 5

Motor motionEngine(FORWARD_ENGINE, BACKWARD_ENGINE);

Motor brush(BRUSH_ENGINE);
Motor valve(VALVE_ENGINE);

Power power(ON_OFF, EMERGENCY_BUTTON);
Led red(INDICATIVE_RED);
Led blue(INDICATIVE_BLUE);
Led green(INDICATIVE_GREEN);
Operation operation;

bool emergencyMode;
bool powerFake = true;

void setup()
{
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(EMERGENCY_BUTTON), emergencyTrigger, CHANGE);
    initialConfiguration();
}

void loop()
{
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
    // delay(1000);
}

void initialConfiguration()
{
    operation.configLeds(red, blue, green);
    operation.configMotors(motionEngine, brush, valve);
}

void emergencyTrigger()
{
    emergencyMode = true;
    power.off();
    Serial.println("--- | EMERGENCY BUTTON WAS TRIGGERED |");
    Serial.println("--- | TURNED OFF OPERATIONS |");
}