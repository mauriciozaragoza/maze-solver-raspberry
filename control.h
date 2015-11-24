#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>

const int LEFT_MOTOR = 20;
const int RIGHT_MOTOR = 21;

using namespace std;

class Control
{
public:
    Control();
    void forward();
    void left();
    void right();
    void stop();
};

#endif
