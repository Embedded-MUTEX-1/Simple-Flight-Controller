#include "AltitudeSensor.h"
#include <cstdint>

AltitudeSensor::AltitudeSensor() {

}

int AltitudeSensor::init() {
    sensor->set_blocking(false);
    speedSmooth.begin(SMOOTHED_AVERAGE, 3);
    return 0;
}

int AltitudeSensor::update() {
    static char i = 0;
    char j = 0;
    int checksum = 0; 
    static int rx[9];
    while(available()) {  
        rx[i] = read();
        if(rx[0] != 0x59) {
            i = 0;
        } else if(i == 1 && rx[1] != 0x59) {
            i = 0;
        } else if(i == 8) {
            for(j = 0; j < 8; j++) {
                checksum += rx[j];
            }
            if(rx[8] == (checksum % 256)) {
                values.alt = rx[2] + rx[3] * 256;
                values.signal_strength = rx[4] + rx[5] * 256;
            }
            i = 0;
        } else {
            i++;
        } 
    }  
    return 0;
}
void AltitudeSensor::getData(struct altitudeData *data) {
    values.vertical_speed = float(values.alt - prevDistance) / 0.05f;

    speedSmooth.add(values.vertical_speed);
    values.vertical_speed = speedSmooth.get();

    prevDistance = values.alt;
    
    *data = values; 
}

bool AltitudeSensor::available() {
    return sensor->readable();
}

char AltitudeSensor::read() {
    char data;
    sensor->read(&data, 1);
    return data;
}

void AltitudeSensor::write(char * data, uint16_t len) {
    sensor->write(data, len);
}