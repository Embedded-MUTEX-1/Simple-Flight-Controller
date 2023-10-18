#ifndef SIMPLE_FLIGHT_CONTROLLER_ALTITUDE_SENSOR_H
#define SIMPLE_FLIGHT_CONTROLLER_ALTITUDE_SENSOR_H

#include "data.h"
#include "mbed.h"
#include "Smoothed.h"
#include <cstdint>
#include <stdint.h>

class AltitudeSensor {
private:
    struct altitudeData values;
    int16_t prevDistance = 0;
    BufferedSerial* sensor = new BufferedSerial(PA_11, PA_12, 115200);
    Smoothed <float> speedSmooth;
public:
    AltitudeSensor();
    int init();
    int update();
    void getData(struct altitudeData* data);
private:
    bool available();
    char read();
    void write(char * data, uint16_t len);
};

#endif //SIMPLE_FLIGHT_CONTROLLER_MONITORING_H