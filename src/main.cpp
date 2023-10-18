/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ThisThread.h"
#include "cmsis_os.h"
#include "mbed.h"
#include "tasks/tasks.h"



int main(int argc, char *argv[])
{
    initTasksAndQueues();
    while (true) {
        ThisThread::sleep_for(osWaitForever);
    }
}