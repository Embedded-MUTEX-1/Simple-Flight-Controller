#ifndef SIMPLE_FLIGHT_CONTROLLERPID_CONTROLLER_H
#define SIMPLE_FLIGHT_CONTROLLERPID_CONTROLLER_H

#include <cstdint>
#include <vector>
#include "pid_lib/FastPID.h"

using namespace std;

class PID_Controller {
private:
    vector<FastPID*> pids;

public:
    PID_Controller(uint8_t number);
    void setConfig(uint8_t pid_index, float p, float i, float d, float freq);
    void getConfig(uint8_t pid_index, float &p, float &i, float &d);
    void setLimit(uint8_t pid_index, int min, int max);
    uint16_t compute(uint8_t pid_index, int setpoint, int input);
    void reset(uint8_t pid_index);
    float computeSetpoint(float angle, uint16_t channel_pulse);
};

#endif