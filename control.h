#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>

// #include <wiringPi.h>

using namespace std;

class Control
{
public:
    void forward();
    void left();
    void right();
    void stop();
};

#endif
