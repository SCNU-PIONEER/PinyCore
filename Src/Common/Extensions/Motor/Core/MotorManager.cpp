#include "MotorManager.hpp"

#include "IMotor.hpp"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace PINYMOTOR;

MotorManager *MotorManager::instance()
{
    static MotorManager instance;
    return &instance;
}

void MotorManager::ctrlTask()
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        for (const auto &motorPair : motorList_) {
            IMotor *motor = motorPair.second;
            motor->update();
        }
        vTaskDelayUntil(&xLastWakeTime,
                        static_cast<TickType_t>(1000.f / this->motorTaskFreq_));
    }
}

void MotorManager::init() {}

uint8_t MotorManager::assignId() { return registedNum_++; }

void MotorManager::taskCreate()
{
    xTaskCreate(
            [](void *_param) -> void { MotorManager::instance()->ctrlTask(); },
            "motor_task", 256, nullptr, osPriorityRealtime, nullptr);
}
