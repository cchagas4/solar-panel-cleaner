#include "Operation.h"

Operation::Operation()
{
}

void Operation::control()
{
    Operation::status = 3; // STARTING
    while (true)
    {
        switch (Operation::status)
        {
        case 0: // "OFF"
            Led::blinkCustom(0, 1000);
            break;
        case 1: // "ON"
            Led::blinkCustom(0, 1000);
            break;
        case 2: // "CLEANING"
            Led::fade();
            break;
        case 3: // "STARTING"
            Led::blinkCustom(400, 400);
            break;
        case 4: // "ERROR"
            Led::blinkCustom(800, 200);
            break;
        default:
            status = 4;
            break;
        }
    }
}