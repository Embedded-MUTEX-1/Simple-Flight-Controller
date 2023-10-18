#include "Motors.h"
#include "mbed.h"
#include "data.h"

Motors::Motors() {
    // mot[0] = new PwmOut(PB_1);
    // mot[1] = new PwmOut(PB_0);
    // mot[2] = new PwmOut(PA_7);
    // mot[3] = new PwmOut(PA_6);
    mot[0] = new PwmOut(PB_1);
    mot[1] = new PwmOut(PB_0);
    mot[2] = new PwmOut(PA_7);
    mot[3] = new PwmOut(PA_6);
    mot[4] = new PwmOut(PA_5);
    mot[5] = new PwmOut(PA_1);
}

void Motors::init() {
    mot[0]->period_us(2500);
    mot[1]->period_us(2500);
    mot[2]->period_us(2500);
    mot[3]->period_us(2500);
    mot[4]->period_us(2500);
    mot[5]->period_us(2500);

    mot[0]->pulsewidth_us(1000);
    mot[1]->pulsewidth_us(1000);
    mot[2]->pulsewidth_us(1000);
    mot[3]->pulsewidth_us(1000);
    mot[4]->pulsewidth_us(1000);
    mot[5]->pulsewidth_us(1000);
}

void Motors::setSpeed(struct controllerData values) {
    for (int i = 0; i < 6; i++) {
        mot[i]->pulsewidth_us(values.mot[i]);
    }
}