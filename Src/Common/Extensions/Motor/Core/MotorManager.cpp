#include "MotorManager.hpp"

#include "IMotor.hpp"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace PINYMOTOR;

void MotorManager::ctrlTask()
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        for (const auto &motorPair : motors()) {
            IMotor *motor = motorPair.second;
            motor->ctrl();
        }
        vTaskDelayUntil(&xLastWakeTime, (1000.f / this->motorTaskFreq_));
    }
}

void MotorManager::taskCreate()
{
    xTaskCreate(
            [](void *param) -> void { MotorManager::instance()->ctrlTask(); },
            "motor_task", 256, NULL, osPriorityRealtime, NULL);
}
