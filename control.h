#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>

const int LEFT_ENABLE = 11;
const int LEFT_MOTOR = 10;
const int RIGHT_ENABLE = 14;
const int RIGHT_MOTOR = 13;

const int PWM_RATE = 25;

using namespace std;

class Control
{
public:
    Control();
    ~Control();
    void forward();
    void left();
    void right();
    void stop();
};

#endif
