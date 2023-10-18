#ifndef SIMPLE_FLIGHT_BATTERY_H
#define SIMPLE_FLIGHT_BATTERY_H

#include "AnalogIn.h"
#include "mbed.h"
#include "data.h"
#include <cstdint>

class Battery {
public:
    Battery();
    int init();
    float getBatteryVoltage();
private:
    AnalogIn *adc = new AnalogIn(PA_0);
};

#endif