#include "MotorManager.hpp"

#include "IMotor.hpp"

#include "FreeRTOS.h"
#include "task.h"

using namespace PINYMOTOR;

void MotorManager::task()
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        for (const auto &motorPair : motors()) {
            IMotor *motor = motorPair.second;
            motor->ctrl();
        }
        vTaskDelayUntil(&xLastWakeTime,
                        (1000.f / this->motorTaskFreq_) / portTICK_RATE_MS);
    } 
}
