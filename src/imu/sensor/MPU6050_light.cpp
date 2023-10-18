/* MPU6050_light library for Arduino
 * 
 * Authors: Romain JL. Fétick (github.com/rfetick)
 *              simplifications and corrections
 *          Tockn (github.com/tockn)
 *              initial author (v1.5.2)
 */

#include "MPU6050_light.h"

/* INIT and BASIC FUNCTIONS */

MPU6050::MPU6050(I2C* w){
  wire = w;
  setGyroOffsets(0,0,0);
  setAccOffsets(0,0,0);
}

uint8_t MPU6050::begin(int gyro_config_num, int acc_config_num){
  // changed calling register sequence [https://github.com/rfetick/MPU6050_light/issues/1] -> thanks to augustosc
  wire->frequency(400000);
  uint8_t status = writeData(MPU6050_PWR_MGMT_1_REGISTER, 0x01); // check only the first connection with status
  writeData(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeData(MPU6050_CONFIG_REGISTER, 0x00);
  setGyroConfig(gyro_config_num);
  setAccConfig(acc_config_num);
  
  this->update();
  return status;
}

uint8_t MPU6050::writeData(uint8_t reg, uint8_t data){
    char buf[2];
    buf[0] = (char)reg;
    buf[1] = (char)data;
    if(wire->write(address << 1, buf, 2, false) != 0) {
        return -1;
    }
  return 0; // 0 if success
}

// This method is not used internaly, maybe by user...
uint8_t MPU6050::readData(uint8_t reg) {
  uint8_t data;
  wire->write(address << 1, (const char *)&reg, 1, false);
  wire->read(address << 1, (char *)&data, 1);
  return data;
}

/* SETTER */

uint8_t MPU6050::setGyroConfig(int config_num){
  uint8_t status;
  switch(config_num){
    case 0: // range = +- 250 deg/s
	  gyro_lsb_to_degsec = 131.0;
	  status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x00);
	  break;
	case 1: // range = +- 500 deg/s
	  gyro_lsb_to_degsec = 65.5;
	  status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x08);
	  break;
	case 2: // range = +- 1000 deg/s
	  gyro_lsb_to_degsec = 32.8;
	  status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x10);
	  break;
	case 3: // range = +- 2000 deg/s
	  gyro_lsb_to_degsec = 16.4;
	  status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x18);
	  break;
	default: // error
	  status = 1;
	  break;
  }
  return status;
}

uint8_t MPU6050::setAccConfig(int config_num){
  uint8_t status;
  switch(config_num){
    case 0: // range = +- 2 g
	  acc_lsb_to_g = 16384.0;
	  status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x00);
	  break;
	case 1: // range = +- 4 g
	  acc_lsb_to_g = 8192.0;
	  status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x08);
	  break;
	case 2: // range = +- 8 g
	  acc_lsb_to_g = 4096.0;
	  status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x10);
	  break;
	case 3: // range = +- 16 g
	  acc_lsb_to_g = 2048.0;
	  status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x18);
	  break;
	default: // error
	  status = 1;
	  break;
  }
  return status;
}

void MPU6050::setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::setAccOffsets(float x, float y, float z){
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

/* CALC OFFSET */

void MPU6050::calcOffsets(bool is_calc_gyro, bool is_calc_acc){
  if(is_calc_gyro){ setGyroOffsets(0,0,0); }
  if(is_calc_acc){ setAccOffsets(0,0,0); }
  float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro
  
  for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
    this->fetchData();
	ag[0] += accX;
	ag[1] += accY;
	ag[2] += (accZ-1.0);
	ag[3] += gyroX;
	ag[4] += gyroY;
	ag[5] += gyroZ;
	ThisThread::sleep_for(1ms); // wait a little bit between 2 measurements
  }
  
  if(is_calc_acc){
    accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
    accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
    accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
  }
  
  if(is_calc_gyro){
    gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
  }
}

/* UPDATE */

void MPU6050::fetchData(){
  char buf[14]; // [ax,ay,az,temp,gx,gy,gz]
  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]
  char regAddr = MPU6050_ACCEL_OUT_REGISTER;

  wire->write(address << 1, (const char *)&regAddr, 1, false);
  wire->read(address << 1, buf, 14);
        
  rawData[0] = ((int16_t)buf[0])  << 8 | buf[1];
  rawData[1] = ((int16_t)buf[2])  << 8 | buf[3];
  rawData[2] = ((int16_t)buf[4])  << 8 | buf[5];
  rawData[3] = ((int16_t)buf[6])  << 8 | buf[7];
  rawData[4] = ((int16_t)buf[8])  << 8 | buf[9];
  rawData[5] = ((int16_t)buf[10]) << 8 | buf[11];
  rawData[6] = ((int16_t)buf[12]) << 8 | buf[13];

  accX = ((float)rawData[0]) / acc_lsb_to_g - accXoffset;
  accY = ((float)rawData[1]) / acc_lsb_to_g - accYoffset;
  accZ = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) / acc_lsb_to_g - accZoffset;
  temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawData[4]) / gyro_lsb_to_degsec - gyroXoffset;
  gyroY = ((float)rawData[5]) / gyro_lsb_to_degsec - gyroYoffset;
  gyroZ = ((float)rawData[6]) / gyro_lsb_to_degsec - gyroZoffset;
}

void MPU6050::update(){
  
}
