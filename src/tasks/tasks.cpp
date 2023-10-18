//
// Created by lenny on 06/08/2023.
//

#include "tasks.h"
#include "Kernel.h"
#include "ThisThread.h"
#include "Thread.h"
#include "data.h"
#include "mbed.h"
#include <cstdint>

const float PID_Roll[3]  = {1.4, 0.001, 0};
const float PID_Pitch[3] = {1.4, 0.001, 0};
const float PID_Yaw[3]   = {4, 0, 0};
const float PID_Alt[3]   = {1.4, 0, 0};

/* Coefficients de mixage des moteurs */

const float C_ROLL_FRONT_REAR = 1.25;
const float C_ROLL_LEFT_RIGHT = 1.5;
const float C_PITCH_FRONT_REAR = 1.5;
const float C_YAW = 1.333;

Queue<struct controllerData, 10> controllerToMonitoringQueue;
Queue<struct controllerData, 10> monitoringToControllerQueue;

Queue<struct receiverData, 10> receiverToMonitoringQueue;
Queue<struct receiverData, 10> receiverToControllerQueue;

Queue<struct altitudeData, 10> altitudeToControllerQueue;
Queue<struct altitudeData, 10> altitudeToMonitoringQueue;

Queue<struct batteryData, 10> batteryToMonitoringQueue;

Thread threadAltitude(osPriorityHigh, OS_STACK_SIZE, nullptr, "altitude");
Thread threadController(osPriorityHigh, OS_STACK_SIZE * 2, nullptr, "controller");
Thread threadReceiver(osPriorityHigh, OS_STACK_SIZE, nullptr, "receiver");
Thread threadMonitoring(osPriorityNormal, OS_STACK_SIZE * 4, nullptr, "monitoring");
Thread threadBattery(osPriorityNormal, OS_STACK_SIZE, nullptr, "battery");

void initTasksAndQueues() {
    threadAltitude.start(altitudeSensorTask);
    threadController.start(controllerTask);
    threadReceiver.start(receiverTask);
    threadMonitoring.start(monitoringTask);
    threadBattery.start(batteryMonitorTask);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void altitudeSensorTask() {
    AltitudeSensor altitudeSensor;

    struct altitudeData values1;
    struct altitudeData values2;

    altitudeSensor.init();
    
    while(true) {
        altitudeSensor.update();
        altitudeSensor.getData(&values1);

        values2 = values1;

        altitudeToControllerQueue.try_put(&values1);
        altitudeToMonitoringQueue.try_put(&values2);

        ThisThread::sleep_for(50ms);
    }
}

void controllerTask() {
    Motors motors;
    IMU imu;
    PID_Controller pid_controller(4);

    struct controllerData values;
    struct controllerData* cmdValues = nullptr;
    struct receiverData* setpoints = nullptr;
    struct altitudeData* altitudeValues = nullptr;

    /*     sp = setpoint    */
    float sp_roll_rate = 0;
    float sp_pitch_rate = 0;
    float sp_yaw_rate = 0;
    float sp_height_speed = 0;
    uint16_t setpoint_yaw = 0;
    float sp_alt = 0;

    int16_t out_roll = 0;
    int16_t out_pitch = 0;
    int16_t out_yaw = 0;
    int16_t out_alt = 0;
    float offset_yaw = 0;
    /*     Maintien altitude    */
    bool altHoldEnable = false;
    bool isSpAltUpdated = false;

    uint64_t timing = 0;
    uint64_t duration = 0;
    uint16_t mot[6] = {0, 0, 0, 0, 0, 0};

    values.mot = mot;

    ThisThread::sleep_for(3000ms);

    motors.init();

    if(imu.init() < 0) {
        values.status = "Error IMU";
        controllerToMonitoringQueue.try_put(&values);
        while(1);
    }

    pid_controller.setConfig(PID_ROLL,  PID_Roll[0],  PID_Roll[1],  PID_Roll[2],  FREQ);
    pid_controller.setConfig(PID_PITCH, PID_Pitch[0], PID_Pitch[1], PID_Pitch[2], FREQ);
    pid_controller.setConfig(PID_YAW,   PID_Yaw[0],   PID_Yaw[1],   PID_Yaw[2],   FREQ);
    pid_controller.setConfig(PID_ALT,   PID_Alt[0],   PID_Alt[1],   PID_Alt[2],   FREQ);

    pid_controller.setLimit(PID_ROLL,  -400, 400);
    pid_controller.setLimit(PID_PITCH, -400, 400);
    pid_controller.setLimit(PID_YAW,   -400, 400);
    pid_controller.setLimit(PID_ALT,   -400, 400);

    while(true) 
    {
        timing = Kernel::get_ms_count();
        
        receiverToControllerQueue.try_get(&setpoints);
        altitudeToControllerQueue.try_get(&altitudeValues);

        /* Réception d'une commande */
        if(monitoringToControllerQueue.try_get(&cmdValues)) {
            pid_controller.setConfig(PID_ROLL,  cmdValues->proll,   cmdValues->iroll,   cmdValues->droll,  FREQ);
            pid_controller.setConfig(PID_PITCH, cmdValues->ppitch,  cmdValues->ipitch,  cmdValues->dpitch, FREQ);
            pid_controller.setConfig(PID_YAW,   cmdValues->pyaw,    cmdValues->iyaw,    cmdValues->dyaw,   FREQ);
            imu.setConfig(cmdValues);
        }

        pid_controller.getConfig(PID_ROLL,  values.proll,   values.iroll,   values.droll);
        pid_controller.getConfig(PID_PITCH, values.ppitch,  values.ipitch,  values.dpitch);
        pid_controller.getConfig(PID_YAW,   values.pyaw,    values.iyaw,    values.dyaw);

        setpoint_yaw = map(setpoints->chan[YAW], 1000, 2000, 2000, 1000);

        values.i2cTiming = Kernel::get_ms_count();
        imu.update();
        values.i2cTiming = Kernel::get_ms_count() - values.i2cTiming;

        values.roll  = imu.getRoll();
        values.pitch = imu.getPitch();
        values.yaw   = imu.getYaw();

        values.gyroRateRoll  = imu.getGyroRateRoll();
        values.gyroRatePitch = imu.getGyroRatePitch();
        values.gyroRateYaw   = imu.getGyroRateYaw();

        values.accRateRoll  = imu.getAccRateRoll();
        values.accRatePitch = imu.getAccRatePitch();
        values.accRateYaw   = imu.getAccRateYaw();

        values.param1 = imu.getParam1();
        values.param2 = imu.getParam2();

        if(setpoints->chan[4] == 2000) {
            altHoldEnable = true;
        } else {
            altHoldEnable = false;
            isSpAltUpdated = false;
        }

        if(setpoints->chan[THROTTLE] < 1100) {
            values.mot[0] = 1000;
            values.mot[1] = 1000;
            values.mot[2] = 1000;
            values.mot[3] = 1000;
            values.mot[4] = 1000;
            values.mot[5] = 1000;

            offset_yaw = values.yaw;
    
            pid_controller.reset(PID_ROLL);
            pid_controller.reset(PID_PITCH);
            pid_controller.reset(PID_YAW);
            pid_controller.reset(PID_ALT);
        }
        else 
        {
            if(altHoldEnable == true && isSpAltUpdated == false) {
                sp_alt = altitudeValues->alt;
                isSpAltUpdated = true;
            }

            sp_height_speed = (sp_alt - altitudeValues->alt) * 2;

            sp_roll_rate = pid_controller.computeSetpoint(values.roll, setpoints->chan[ROLL]);
            sp_pitch_rate = pid_controller.computeSetpoint(values.pitch, setpoints->chan[PITCH]);
            //sp_yaw_rate = pid_controller.computeSetpoint(values.yaw, setpoint_yaw);
            
            if(!(setpoint_yaw > 1450 && setpoint_yaw < 1550)) {
                sp_yaw_rate = pid_controller.computeSetpoint(0, setpoint_yaw);
                offset_yaw = values.yaw;
            } else {
                values.yaw -= offset_yaw;
                sp_yaw_rate = pid_controller.computeSetpoint(values.yaw, setpoint_yaw);
            }
            
            out_roll = pid_controller.compute(PID_ROLL, sp_roll_rate, values.gyroRateRoll);
            out_pitch = pid_controller.compute(PID_PITCH, sp_pitch_rate, values.gyroRatePitch);
            out_yaw = pid_controller.compute(PID_YAW, sp_yaw_rate, values.gyroRateYaw);
            out_alt = pid_controller.compute(PID_ALT, sp_height_speed, altitudeValues->vertical_speed);

            if(!(setpoints->chan[ROLL] > 1300 && setpoints->chan[ROLL] < 1700 && setpoints->chan[PITCH] > 1300 && setpoints->chan[PITCH] < 1700) || !altHoldEnable)
                out_alt = 0;

            values.mot[0] = setpoints->chan[THROTTLE] + int16_t(out_roll != 0 ? out_roll * C_ROLL_LEFT_RIGHT : 0)                                                                - int16_t(out_yaw != 0 ? out_yaw * C_YAW : 0) + out_alt;
            values.mot[1] = setpoints->chan[THROTTLE] + int16_t(out_roll != 0 ? out_roll * C_ROLL_FRONT_REAR : 0) - int16_t(out_pitch != 0 ? out_pitch * C_PITCH_FRONT_REAR : 0) + int16_t(out_yaw != 0 ? out_yaw * C_YAW : 0) + out_alt;
            values.mot[2] = setpoints->chan[THROTTLE] - int16_t(out_roll != 0 ? out_roll * C_ROLL_FRONT_REAR : 0) - int16_t(out_pitch != 0 ? out_pitch * C_PITCH_FRONT_REAR : 0) - int16_t(out_yaw != 0 ? out_yaw * C_YAW : 0) + out_alt;
            values.mot[3] = setpoints->chan[THROTTLE] - int16_t(out_roll != 0 ? out_roll * C_ROLL_LEFT_RIGHT : 0)                                                                + int16_t(out_yaw != 0 ? out_yaw * C_YAW : 0) + out_alt;
            values.mot[4] = setpoints->chan[THROTTLE] - int16_t(out_roll != 0 ? out_roll * C_ROLL_FRONT_REAR : 0) + int16_t(out_pitch != 0 ? out_pitch * C_PITCH_FRONT_REAR : 0) - int16_t(out_yaw != 0 ? out_yaw * C_YAW : 0) + out_alt;
            values.mot[5] = setpoints->chan[THROTTLE] + int16_t(out_roll != 0 ? out_roll * C_ROLL_FRONT_REAR : 0) + int16_t(out_pitch != 0 ? out_pitch * C_PITCH_FRONT_REAR : 0) + int16_t(out_yaw != 0 ? out_yaw * C_YAW : 0) + out_alt;
        }

        if(setpoints->chan[5] == 2000 || cmdValues->status.compare("disarmed") == 0) {
            values.mot[0] = 1000;
            values.mot[1] = 1000;
            values.mot[2] = 1000;
            values.mot[3] = 1000;
            values.mot[4] = 1000;
            values.mot[5] = 1000;
            values.status = "DISARMED";
        } else {
            values.status = "ARMED";
        }

        if(cmdValues->mode.compare("manu") == 0 && values.status.compare("ARMED") == 0)
            motors.setSpeed(*cmdValues);
        else
            motors.setSpeed(values);

        duration = Kernel::get_ms_count() - timing;

        for(uint64_t i = 0;i < (4 - duration);i++) {
            ThisThread::sleep_for(1ms);
        }

        timing = Kernel::get_ms_count()- timing;

        values.loopRate = timing;

        controllerToMonitoringQueue.try_put(&values);
    }
}

void monitoringTask() {
    Monitoring monitoring;

    struct controllerData cmdController;
    struct controllerData* valuesController = nullptr;
    struct receiverData* setpoints = nullptr;
    struct altitudeData* valuesAltitude = nullptr;
    struct batteryData* valuesBattery = nullptr;

    DigitalOut status(PC_14);
    DigitalOut fault(PC_15);

    while(true) {
        receiverToMonitoringQueue.try_get(&setpoints);
        controllerToMonitoringQueue.try_get(&valuesController);
        altitudeToMonitoringQueue.try_get(&valuesAltitude);
        batteryToMonitoringQueue.try_get(&valuesBattery);

        ThisThread::sleep_for(10ms);

        if(setpoints == nullptr || valuesController == nullptr || valuesAltitude == nullptr || valuesBattery == nullptr) {
            continue;
        }
        

        if(monitoring.isDataAvailable()) {
            monitoring.readCommands(&cmdController);
            monitoringToControllerQueue.try_put(&cmdController);
        }

        monitoring.sendValuesToClient(*valuesController, *setpoints, *valuesAltitude, *valuesBattery, "TEST");
        
    }
}

void receiverTask() {
    IBusBM receiver;

    struct receiverData setpoints1 = {"init", 1000, 1000, 1000, 1000, 1000};
    struct receiverData setpoints2 = {"init", 1000, 1000, 1000, 1000, 1000};

    BufferedSerial serial(PA_9, PA_10, 115200);
    receiver.begin(serial);

    while (true) {
        receiver.loop();

        for (int i = 0; i < 6; i++) {
            setpoints1.chan[i] = receiver.readChannel(i);
            setpoints2.chan[i] = receiver.readChannel(i);
        }

        if(setpoints1.chan[4] == 2000 && setpoints1.chan[5] == 2000)
            setpoints1.status = "Receiver error";
        else
            setpoints1.status = "Receiver OK";

        receiverToMonitoringQueue.try_put(&setpoints1);
        receiverToControllerQueue.try_put(&setpoints2);

        ThisThread::sleep_for(10ms);
    }
}

void batteryMonitorTask() {
    Battery battery;

    struct batteryData batteryValues = {"init", 0.0f};
    int status = 0;

    if(battery.init() < 0) {
        batteryValues.status = "error init battery";
        batteryToMonitoringQueue.try_put(&batteryValues);
    }

    while(true) {

        batteryValues.vBat = battery.getBatteryVoltage();

        if(battery.getBatteryVoltage() < 10.5)
            batteryValues.status = "Under voltage Vbat";
        else 
            batteryValues.status = "Vbat OK";

        batteryToMonitoringQueue.try_put(&batteryValues);

        ThisThread::sleep_for(50ms);
    }
}