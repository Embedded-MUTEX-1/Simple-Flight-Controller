#ifndef SIMPLE_FLIGHT_MOTORS_H
#define SIMPLE_FLIGHT_MOTORS_H

#include "mbed.h"
#include "data.h"

class Motors {
private:
    PwmOut* mot[6];
public:
    Motors();
    void init();
    void setSpeed(struct controllerData values);
};

#endif