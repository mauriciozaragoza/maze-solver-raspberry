#include <wiringPi.h>

#include "control.h"

Control::Control()
{
    pinMode(RIGHT_MOTOR, OUTPUT);
    pinMode(LEFT_MOTOR, OUTPUT);
}

void Control::forward()
{
    digitalWrite(RIGHT_MOTOR, HIGH);
    digitalWrite(LEFT_MOTOR, HIGH);
}

void Control::right()
{
    digitalWrite(RIGHT_MOTOR, LOW);
    digitalWrite(LEFT_MOTOR, HIGH);
}

void Control::left()
{
    digitalWrite(RIGHT_MOTOR, HIGH);
    digitalWrite(LEFT_MOTOR, LOW);
}

void Control::stop()
{
    digitalWrite(RIGHT_MOTOR, LOW);
    digitalWrite(LEFT_MOTOR, LOW);
}