//
// Created by lenny on 06/08/2023.
//

#include "Monitoring.h"
#include "document.h"

const char* statusList[] = {"ARMED", "DISARMED", "ERR", "ERR_CMD", "LOW_BAT", "AUTO", "MANU"};

void Monitoring::init() {

}

void Monitoring::dataAvailableFromClient() {

    isReading = true;

    char c = readData();

    if (c == '\n' || jsonIndex >= sizeof(jsonBuffer) - 1) {
        // Fin de la chaîne JSON ou buffer plein
        jsonBuffer[jsonIndex] = '\0'; // Terminer la chaîne avec un caractère nul
        jsonIndex = 0; // Réinitialiser l'index pour la prochaine lecture
        dataAvailable = true;
        isReading = false;
    } else {
        // Ajouter le caractère au tableau
        jsonBuffer[jsonIndex++] = c;
    }
}

void Monitoring::sendValuesToClient(struct controllerData values1, struct receiverData values2, struct altitudeData values4, struct batteryData values5, std::string status) {

    ParseResult result = documentTx.Parse(jsonBodyOutput);

    if(!result)
        return;

    documentTx["roll"].SetFloat(values1.roll);
    documentTx["pitch"].SetFloat(values1.pitch);
    documentTx["yaw"].SetFloat(values1.yaw);

    documentTx["gyroRoll"].SetFloat(values1.gyroRateRoll);
    documentTx["gyroPitch"].SetFloat(values1.gyroRatePitch);
    documentTx["gyroYaw"].SetFloat(values1.gyroRateYaw);

    documentTx["accRoll"].SetFloat(values1.accRateRoll);
    documentTx["accPitch"].SetFloat(values1.accRatePitch);
    documentTx["accYaw"].SetFloat(values1.accRateYaw);

    documentTx["proll"].SetFloat(values1.proll);
    documentTx["ppitch"].SetFloat(values1.ppitch);
    documentTx["pyaw"].SetFloat(values1.pyaw);

    documentTx["iroll"].SetFloat(values1.iroll);
    documentTx["ipitch"].SetFloat(values1.ipitch);
    documentTx["iyaw"].SetFloat(values1.iyaw);

    documentTx["droll"].SetFloat(values1.droll);
    documentTx["dpitch"].SetFloat(values1.dpitch);
    documentTx["dyaw"].SetFloat(values1.dyaw);

    documentTx["param1"].SetFloat(values1.param1);
    documentTx["param2"].SetFloat(values1.param2);

    documentTx["i2cErrors"].SetInt(values1.i2cTiming);
    documentTx["loopTime"].SetInt(values1.loopRate);
    documentTx["alt"].SetInt(values4.alt);
    documentTx["vBat"].SetFloat(values5.vBat);

    values4.status = "Altitude OK";

    status = values1.status; //+ " : " + values2.status + " : " + values3.status + " : " + values4.status + " : " + values5.status;

    documentTx["status"].SetString(status.c_str(), status.length());

    rapidjson::Value motArray(rapidjson::kArrayType);
    rapidjson::Value chArray(rapidjson::kArrayType);

    for (int i = 0;i < 6;i++) {
        rapidjson::Value motValue;
        motValue.SetInt(values1.mot[i]);
        motArray.PushBack(motValue, documentTx.GetAllocator());

        rapidjson::Value chValue;
        chValue.SetInt(values2.chan[i]);
        chArray.PushBack(chValue, documentTx.GetAllocator());
    }

    auto memberMot = documentTx.FindMember("mot");
    memberMot->value = motArray;

    auto memberCh = documentTx.FindMember("ch");
    memberCh->value = chArray;

    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    documentTx.Accept(writer);

    writeData((char *)buffer.GetString());

    documentTx.GetAllocator().Clear();
}

void Monitoring::readCommands(struct controllerData *cmdController) {

    ParseResult result = documentRx.Parse(jsonBuffer, strlen(jsonBuffer));

    memset(jsonBuffer, '\0', sizeof(jsonBuffer));
    dataAvailable = false;

    if(!result) {
        documentRx.GetAllocator().Clear();
        return;
    }

    cmdController->roll = documentRx["roll"].GetFloat();
    cmdController->pitch = documentRx["pitch"].GetFloat();
    cmdController->yaw = documentRx["yaw"].GetFloat();

    cmdController->gyroRateRoll  = documentRx["gyroRoll"].GetFloat();
    cmdController->gyroRatePitch = documentRx["gyroPitch"].GetFloat();
    cmdController->gyroRateYaw   = documentRx["gyroYaw"].GetFloat();

    cmdController->accRateRoll = documentRx["accRoll"].GetFloat();
    cmdController->accRatePitch = documentRx["accPitch"].GetFloat();
    cmdController->accRateYaw = documentRx["accYaw"].GetFloat();

    cmdController->proll = documentRx["proll"].GetFloat();
    cmdController->ppitch = documentRx["ppitch"].GetFloat();
    cmdController->pyaw = documentRx["pyaw"].GetFloat();

    cmdController->iroll = documentRx["iroll"].GetFloat();
    cmdController->ipitch = documentRx["ipitch"].GetFloat();
    cmdController->iyaw = documentRx["iyaw"].GetFloat();

    cmdController->droll = documentRx["droll"].GetFloat();
    cmdController->dpitch = documentRx["dpitch"].GetFloat();
    cmdController->dyaw = documentRx["dyaw"].GetFloat();

    cmdController->param1 = documentRx["param1"].GetFloat();
    cmdController->param2 = documentRx["param2"].GetFloat();

    cmdController->status = (std::string)documentRx["status"].GetString();
    cmdController->mode = (std::string)documentRx["mode"].GetString();

    Value& value2 = documentRx["mot"].GetArray();
    for(int i = 0;i < 6;i++)
        tempValues[i] = value2[i].GetUint();

    cmdController->mot = tempValues;

    documentRx.GetAllocator().Clear();

    return;
}

void Monitoring::writeData(char *str) {

    this->write(str, strlen(str));
    this->write("\n", 1);
}

char Monitoring::readData() {
    char c;
    this->read(&c, 1);
    return c;
}

Monitoring::Monitoring(PinName tx, PinName rx, int baud) 
: UnbufferedSerial(tx, rx, baud) {
    jsonBodyOutput = "{\"ch\":[1000,1000,1000,1000,1000,1000],\"i2cErrors\":0,\"loopTime\":0,\"alt\":0,\"mot\":[1000,1000,1000,1000,1000,1000],\"roll\":0,\"pitch\":0,\"yaw\":0,\"gyroRoll\":0,\"gyroPitch\":0,\"gyroYaw\":0,\"accRoll\":0,\"accPitch\":0,\"accYaw\":0,\"proll\":0,\"ppitch\":0,\"pyaw\":0,\"iroll\":0,\"ipitch\":0,\"iyaw\":0,\"droll\":0,\"dpitch\":0,\"dyaw\":0,\"status\":\"na\",\"param1\":0,\"param2\":0,\"vBat\":0.0}";
    this->attach(callback(this, &Monitoring::dataAvailableFromClient) , mbed::SerialBase::RxIrq);
}
