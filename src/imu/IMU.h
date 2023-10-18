#ifndef SIMPLE_FLIGHT_IMU_H
#define SIMPLE_FLIGHT_IMU_H

#include "mbed.h"
#include "data.h"
#include "sensor/MPU6050_light.h"
#include "sensor/MPU9250.h"
#include "sensor/BMI088.h"
#include "mag/DFRobot_BMM150.h"
#include "filter/MahonyAHRS.h"
#include "filter/MadgwickAHRS.h"
#include "filter/IMU_Filter.h"
#include <cstdint>

#define RAD_TO_DEG (180 / 3.14159265358979323846)
#define DEG_TO_RAD (3.14159265358979323846 / 180)

class IMU {
private:
    float gyroRateRoll = 0;
    float gyroRatePitch  = 0;
    float gyroRateYaw = 0;
    float accRateRoll = 0;
    float accRatePitch = 0;
    float accRateYaw = 0;
    int16_t magRateX = 0;
    int16_t magRateY = 0;
    int16_t magRateZ = 0;
    int16_t magOffsetRateX = 0;
    int16_t magOffsetRateY = 0;
    int16_t magOffsetRateZ = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0; 
    float filterGyroCoeff  = 0.999;
    float filterAccelCoeff = 1 - filterGyroCoeff;
    float gyro_roll = 0;
    float gyro_pitch = 0;
    float gyro_yaw = 0;
    float acc_roll = 0;
    float acc_pitch = 0;
    float offsetRoll = 0;
    float offsetPitch = 0;
    float offsetYaw = 0;
    float gyroRateOffsetRoll = 0;
    float gyroRateOffsetPitch = 0;
    float gyroRateOffsetYaw = 0;
    float accRateOffsetRoll = 0;
    float accRateOffsetPitch = 0;
    float accRateOffsetYaw = 0;
    unsigned long prevTime, currentTime, prevTime2 = 0;
    long intervalStart = 0;
    float dt = 0;
    I2C* w = new I2C(PB_7, PB_6);
    MPU9250* mpu;
    Bmi088Accel* accel = new Bmi088Accel(*w, 0x19);
    Bmi088Gyro* gyro = new Bmi088Gyro(*w, 0x69);
    DFRobot_BMM150_I2C* bmm150;
    Mahony filter;

public:
    IMU();
    int init();
    int update();
    void updateInterval();
    void resetValues();
    void setConfig(struct controllerData* imuCmd);

    float getGyroRateRoll() { return gyroRateRoll; }
    float getGyroRatePitch() { return gyroRatePitch; }
    float getGyroRateYaw() { return gyroRateYaw; }
    float getAccRateRoll() { return accRateRoll; }
    float getAccRatePitch() { return accRatePitch; }
    float getAccRateYaw() { return accRateYaw; }
    float getRoll() { return roll; }
    float getPitch() { return pitch; }
    float getYaw() { return yaw; }
    float getParam1();
    float getParam2();

private:
    float wrap(float angle);
    void calibration(int calibNum);
    void calibrationMag(int timeout);
};

#endif //SIMPLE_FLIGHT_CONTROLLER_MONITORING_H