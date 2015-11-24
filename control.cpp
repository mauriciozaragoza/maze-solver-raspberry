#include <wiringPi.h>
#include <stdio.h>
#include <softPwm.h>

#include "control.h"

Control::Control()
{
    wiringPiSetup();

    pinMode(RIGHT_ENABLE, OUTPUT);
    pinMode(LEFT_ENABLE, OUTPUT);
    pinMode(RIGHT_MOTOR, OUTPUT);
    pinMode(LEFT_MOTOR, OUTPUT);

    softPwmCreate(LEFT_MOTOR, PWM_RATE, 100);
    softPwmCreate(RIGHT_MOTOR, PWM_RATE, 100);

    // digitalWrite(RIGHT_ENABLE, HIGH);
    // digitalWrite(LEFT_ENABLE, HIGH);
}

Control::~Control()
{
    this->stop();
}

void Control::forward()
{
    digitalWrite(RIGHT_ENABLE, HIGH);
    digitalWrite(LEFT_ENABLE, HIGH);
}

void Control::right()
{
    digitalWrite(RIGHT_ENABLE, LOW);
    digitalWrite(LEFT_ENABLE, HIGH);
}

void Control::left()
{
    digitalWrite(RIGHT_ENABLE, HIGH);
    digitalWrite(LEFT_ENABLE, LOW);
}

void Control::stop()
{
    digitalWrite(RIGHT_ENABLE, LOW);
    digitalWrite(LEFT_ENABLE, LOW);
}