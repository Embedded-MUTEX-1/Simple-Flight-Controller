//
// Created by lenny on 06/08/2023.
//

#ifndef SIMPLE_FLIGHT_CONTROLLER_DATA_H
#define SIMPLE_FLIGHT_CONTROLLER_DATA_H

#define PID_ROLL       0
#define PID_PITCH      1
#define PID_YAW        2
#define PID_ALT        3
#define THROTTLE       2
#define ROLL           0
#define PITCH          1
#define YAW            3
#define FREQ         250
#define HEADER      0x59

#include <cstdint>
#include <string>
#include <iostream>
#include <sstream>

enum droneStatus {
    ARMED = 0, DISARMED, ERR, ERR_CMD, LOW_BAT, AUTO, MANU
};

enum stateAltitude {
    HEADER_1, HEADER_2, DATA, CHECKSUM
};

struct controllerData {
    std::string status;
    std::string mode;
    uint16_t* mot;
    float proll, ppitch, pyaw;
    float iroll, ipitch, iyaw;
    float droll, dpitch, dyaw;
    float gyroRateRoll;
    float gyroRatePitch;
    float gyroRateYaw;
    float accRateRoll;
    float accRatePitch;
    float accRateYaw;
    float roll;
    float pitch;
    float yaw;
    float param1, param2;
    uint64_t i2cTiming;
    uint64_t loopRate;
};

struct altitudeData {
    std::string status;
    int16_t alt;
    float vertical_speed;
    uint16_t signal_strength;
    float temperature;
};

struct receiverData {
    std::string status;
    uint16_t chan[14];
};

struct batteryData {
    std::string status;
    float vBat;
};

long map(long x, long in_min, long in_max, long out_min, long out_max);
std::string format(float f, int num);

#endif //SIMPLE_FLIGHT_CONTROLLER_DATA_H
