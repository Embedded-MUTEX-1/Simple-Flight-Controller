//
// Created by lenny on 06/08/2023.
//

#ifndef SIMPLE_FLIGHT_CONTROLLER_MONITORING_H
#define SIMPLE_FLIGHT_CONTROLLER_MONITORING_H

#include "data.h"
#include "mbed.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "pid/PID_Controller.h"
#include "imu/IMU.h"
#include <vector> // classe list (conteneur)
#include <cstring>
 
using namespace rapidjson;

class Monitoring : public UnbufferedSerial{
private:
    //BufferedSerial* BTmodule = new BufferedSerial(PA_2, PA_3, 115200);
    Document documentTx;
    Document documentRx;
    const char *jsonBodyOutput;
    int temp;
    char jsonBuffer[750];
    int jsonIndex = 0;
    bool dataAvailable = false;
    bool isReading = false;
    uint16_t tempValues[6] = {0, 0, 0, 0, 0, 0};

public:
    void init();
    void sendValuesToClient(struct controllerData values1, struct receiverData values2, struct altitudeData values4, struct batteryData values5, std::string status);
    void dataAvailableFromClient();
    void readCommands(struct controllerData *cmdController);
    Monitoring(PinName tx = PA_2, PinName rx = PA_3, int baud=115200);
    bool isDataAvailable() {
        return dataAvailable;
    }

private:
    void writeData(char* str);
    char readData();
};


#endif //SIMPLE_FLIGHT_CONTROLLER_MONITORING_H
