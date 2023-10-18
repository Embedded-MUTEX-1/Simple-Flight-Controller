#include "Battery.h"

Battery::Battery() {
    adc->set_reference_voltage(3.3f);
}

int Battery::init() {
    int status = 0;

    if(getBatteryVoltage() > 12.6)
        status = -1;
    else if(getBatteryVoltage() < 9.5)
        status = -2;

    return status;
}

float Battery::getBatteryVoltage() {
    return (12.0f + 1.0f) * adc->read_voltage();
}