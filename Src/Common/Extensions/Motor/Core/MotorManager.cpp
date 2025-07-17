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
            motor->update();
        }
        vTaskDelayUntil(&xLastWakeTime, (1000.f / this->motorTaskFreq_));
    }
}

void MotorManager::init() {}

uint8_t MotorManager::motorListSize()
{
    return static_cast<uint8_t>(motorList_.size());
}

void MotorManager::taskCreate()
{
    xTaskCreate(
            [](void *param) -> void { MotorManager::instance()->ctrlTask(); },
            "motor_task", 256, NULL, osPriorityRealtime, NULL);
}
