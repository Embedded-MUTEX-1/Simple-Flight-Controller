#include "IMU.h"
#include "BMI088.h"
#include "sensor/MPU6050_light.h"
#include <math.h>

IMU::IMU() {

}
int IMU::init() {

    int status = 0;

    filter.begin(250);

    // w->frequency(400000);

    // mpu = new MPU9250();
    // if(!mpu->setup(0x68, MPU9250Setting(), w))
    // {
    //     status = -1;
    // }

    // mpu->calibrateAccelGyro();

    // //mpu = new MPU6050(w);
    w->frequency(400000);
    status = accel->begin();
    status = gyro->begin();

    gyro->setRange(Bmi088Gyro::RANGE_1000DPS);
    accel->setRange(Bmi088Accel::RANGE_6G);

    gyro->setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
    accel->setOdr(Bmi088Accel::ODR_400HZ_BW_40HZ);


    // calibration(1000);

    // if(bmm150->begin() < 0) {
    //     return -1;
    // }

    // calibrationMag(10000);

    // bmm150->setOperationMode(BMM150_POWERMODE_NORMAL);
    // bmm150->setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    // bmm150->setRate(BMM150_DATA_RATE_10HZ);
    // bmm150->setMeasurementXYZ();

    // if(mpu->readData(0x75) != 0x68) {
    //     return -1;
    // }
    // mpu->begin(1, 1);
    // mpu->writeData(MPU6050_CONFIG_REGISTER, 0x04);
    // if(mpu->readData(MPU6050_CONFIG_REGISTER) != 0x04) {
    //     return -1;
    // }
    // ThisThread::sleep_for(100ms);
    // mpu->calcOffsets(true, false);
    // //calibration(5000);
    // intervalStart = Kernel::get_ms_count();

    // return 0;
    return status;
}
int IMU::update() {

    // sBmm150MagData_t magData = bmm150->getGeomagneticData();

    // gyroRateRoll  = magData.x - magOffsetRateX;
    // gyroRatePitch = magData.y - magOffsetRateY;
    // gyroRateYaw   = magData.z - magOffsetRateZ;

    // // mpu->fetchData();
    // mpu->update();

    if(!gyro->isCorrectId() || !accel->isCorrectId())
        return -1;

    gyro->readSensor();
    accel->readSensor();

    gyroRateRoll = gyro->getGyroX_rads() - gyroRateOffsetRoll;
    gyroRatePitch = gyro->getGyroY_rads() - gyroRateOffsetPitch;
    gyroRateYaw = gyro->getGyroZ_rads() - gyroRateOffsetYaw;

    accRateRoll = (accel->getAccelX_mss()) - accRateOffsetRoll;
    accRatePitch = (accel->getAccelY_mss()) - accRateOffsetPitch;
    accRateYaw = (accel->getAccelZ_mss()) - accRateOffsetYaw;

    // gyroRateRoll = gx - gyroRateOffsetRoll;
    // gyroRatePitch = gy - gyroRateOffsetPitch;
    // gyroRateYaw = gz - gyroRateOffsetYaw;

    // accRateRoll = ax;
    // accRatePitch = ay;
    // accRateYaw = az;

    // filter.updateIMU(
    //     gyroRateRoll,
    //     gyroRatePitch,
    //     gyroRateYaw,
    //     accRateRoll,
    //     accRatePitch,
    //     accRateYaw
    // );

    filter.updateIMU(
        gyroRateRoll,
        gyroRatePitch,
        gyroRateYaw,
        accRateRoll,
        accRatePitch,
        accRateYaw
    );

    roll = filter.getRoll() - offsetRoll; //- 1.19f;
    pitch = filter.getPitch() - offsetPitch; //- 1.0f;
    yaw = filter.getYaw() - offsetYaw;

    // // Computing accel angles
	// acc_roll  = wrap((atan2(accRatePitch, sqrt(accRateYaw * accRateYaw + accRateRoll * accRateRoll))) * RAD_TO_DEG);
	// acc_pitch = wrap((-atan2(accRateRoll, sqrt(accRateYaw * accRateYaw + accRatePitch * accRatePitch))) * RAD_TO_DEG);

    // // Computing gyro angles
    // dt = (Kernel::get_ms_count() - intervalStart) * 0.001f;
    // gyro_roll  = wrap(gyro_roll  + gyroRateRoll  * dt);
	// gyro_pitch = wrap(gyro_pitch + gyroRatePitch * dt);
	// gyro_yaw   = wrap(gyro_yaw   + gyroRateYaw   * dt);

    // // Computing complementary filter angles
	// gyro_roll  = gyro_roll  * filterGyroCoeff + acc_roll  * filterAccelCoeff;
    // gyro_pitch = gyro_pitch * filterGyroCoeff + acc_pitch * filterAccelCoeff;

    // roll  = roll  * 0.9f + gyro_roll * 0.1f;
    // pitch = pitch * 0.9f + gyro_pitch * 0.1f;

    return 0;
}
void IMU::updateInterval() {
    intervalStart = Kernel::get_ms_count();
}
void IMU::resetValues() {
    gyro_roll = acc_roll;
    gyro_pitch = acc_pitch;
}
float IMU::wrap(float angle)
{
	while (angle > +180) angle -= 360;
	while (angle < -180) angle += 360;
	return angle;
}
void IMU::calibration(int calibNum)
{
    // float ax = 0, ay = 0, az = 0;
    // float gx = 0, gy = 0, gz = 0;

    // for (size_t i = 0; i < calibNum; i++)
    // {
    //     bmi088->getGyroscope(&gx, &gy, &gz);

    //     gyroRateRoll = gx;
    //     gyroRatePitch = gy;
    //     gyroRateYaw = gz;

    //     gyroRateOffsetRoll += gyroRateRoll;
    //     gyroRateOffsetPitch += gyroRatePitch;
    //     gyroRateOffsetYaw += gyroRateYaw;

    //     ThisThread::sleep_for(1ms);
    // }

    // for (size_t i = 0; i < calibNum; i++)
    // {
    //     bmi088->getAcceleration(&ax, &ay, &az);

    //     accRateRoll = ax;
    //     accRatePitch = ay;
    //     accRateYaw = az;

    //     accRateOffsetRoll += accRateRoll;
    //     accRateOffsetPitch += accRatePitch;
    //     accRateOffsetYaw += accRateYaw - 1;

    //     ThisThread::sleep_for(1ms);
    // }

    // gyroRateOffsetRoll /= calibNum;
    // gyroRateOffsetPitch /= calibNum;
    // gyroRateOffsetYaw /= calibNum;

    // accRateOffsetRoll /= calibNum;
    // accRateOffsetPitch /= calibNum;
    // accRateOffsetYaw /= calibNum;

}

void IMU::calibrationMag(int timeout) 
{
//     int16_t value_x_min = 0;
//     int16_t value_x_max = 0;
//     int16_t value_y_min = 0;
//     int16_t value_y_max = 0;
//     int16_t value_z_min = 0;
//     int16_t value_z_max = 0;
//     uint32_t timeStart = 0;

//     sBmm150MagData_t magData = bmm150->getGeomagneticData();
//     value_x_min = magData.x;
//     value_x_max = magData.x;
//     value_y_min = magData.y;
//     value_y_max = magData.y;
//     value_z_min = magData.z;
//     value_z_max = magData.z;
//     ThisThread::sleep_for(100ms);

//     timeStart = Kernel::get_ms_count();

//     while ((Kernel::get_ms_count() - timeStart) < timeout) {
//         magData = bmm150->getGeomagneticData();

//         /* Update x-Axis max/min value */
//         if (value_x_min > magData.x) {
//             value_x_min = magData.x;
//             // Serial.print("Update value_x_min: ");
//             // Serial.println(value_x_min);

//         } else if (value_x_max < magData.x) {
//             value_x_max = magData.x;
//             // Serial.print("update value_x_max: ");
//             // Serial.println(value_x_max);
//         }

//         /* Update y-Axis max/min value */
//         if (value_y_min > magData.y) {
//             value_y_min = magData.y;
//             // Serial.print("Update value_y_min: ");
//             // Serial.println(value_y_min);

//         } else if (value_y_max < magData.y) {
//             value_y_max = magData.y;
//             // Serial.print("update value_y_max: ");
//             // Serial.println(value_y_max);
//         }

//         /* Update z-Axis max/min value */
//         if (value_z_min > magData.z) {
//             value_z_min = magData.z;
//             // Serial.print("Update value_z_min: ");
//             // Serial.println(value_z_min);

//         } else if (value_z_max < magData.z) {
//             value_z_max = magData.z;
//             // Serial.print("update value_z_max: ");
//             // Serial.println(value_z_max);
//         }

//         ThisThread::sleep_for(100ms);

//     }

//     magOffsetRateX = value_x_min + (value_x_max - value_x_min) / 2;
//     magOffsetRateY = value_y_min + (value_y_max - value_y_min) / 2;
//     magOffsetRateZ = value_z_min + (value_z_max - value_z_min) / 2;
}

void IMU::setConfig(struct controllerData* imuCmd) {

    filter.setConfig(imuCmd->param1, imuCmd->param2);

    offsetRoll = imuCmd->roll;
    offsetPitch = imuCmd->pitch;
    offsetYaw = imuCmd->yaw;

    gyroRateOffsetRoll = imuCmd->gyroRateRoll;
    gyroRateOffsetPitch = imuCmd->gyroRatePitch;
    gyroRateOffsetYaw = imuCmd->gyroRateYaw;

    accRateOffsetRoll = imuCmd->accRateRoll;
    accRateOffsetPitch = imuCmd->accRatePitch;
    accRateOffsetYaw = imuCmd->accRateYaw;
}

float IMU::getParam1() {
    return filter.getParam1();
}

float IMU::getParam2() {
    return filter.getParam2();
}