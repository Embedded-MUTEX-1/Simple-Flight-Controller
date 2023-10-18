#include "PID_Controller.h"
#include <cstdint>

PID_Controller::PID_Controller(uint8_t number) {
    for (uint8_t i = 0;i < number;i++) {
        pids.push_back(new FastPID());
    }
}
void PID_Controller::setConfig(uint8_t pid_index, float p, float i, float d, float freq) {
    FastPID* pid = pids.at(pid_index);
    pid->configure(p, i, d, freq, 16, true);
}
void PID_Controller::getConfig(uint8_t pid_index, float &p, float &i, float &d) {
    FastPID* pid = pids.at(pid_index);
    pid->getConfig(p, i, d);
}
void PID_Controller::setLimit(uint8_t pid_index, int min, int max) {
    FastPID* pid = pids.at(pid_index);
    pid->setOutputRange(min, max);
}
uint16_t PID_Controller::compute(uint8_t pid_index, int setpoint, int input) {
    FastPID* pid = pids.at(pid_index);
    return pid->step(setpoint, input);
}
void PID_Controller::reset(uint8_t pid_index) {
    FastPID* pid = pids.at(pid_index);
    pid->clear();
}
float PID_Controller::computeSetpoint(float angle, uint16_t channel_pulse) {
    float level_adjust = angle * 15; // Value 15 limits maximum angle value to ±10°
    float set_point    = 0;
    // Need a dead band of 16µs for better result
    if (channel_pulse > 1508) {
    set_point = channel_pulse - 1508;
    } else if (channel_pulse <  1492) {
        set_point = channel_pulse - 1492;
    }

    set_point -= level_adjust;
    set_point /= 3;

    return set_point;
}