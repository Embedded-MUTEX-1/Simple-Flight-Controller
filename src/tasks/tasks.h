//
// Created by lenny on 06/08/2023.
//

#ifndef SIMPLE_FLIGHT_CONTROLLER_TASKS_H
#define SIMPLE_FLIGHT_CONTROLLER_TASKS_H

#include "monitoring/Monitoring.h"
#include "receiver/IBusBM.h"
#include "motors/Motors.h"
#include "imu/IMU.h"
#include "pid/PID_Controller.h"
#include "altitude/AltitudeSensor.h"
#include "battery/Battery.h"

#define MAX_CHAN_VALUE 2000
#define MIN_CHAN_VALUE 1000

void initTasksAndQueues();
void altitudeSensorTask();
void controllerTask();
void monitoringTask();
void receiverTask();
void batteryMonitorTask();

#endif //SIMPLE_FLIGHT_CONTROLLER_TASKS_H
