#include "MotorManager.hpp"

#include "IMotor.hpp"

using namespace PINYMOTOR;

void MotorManager::task()
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        for (const auto &motorPair : motorList_) {
            IMotor *motor = motorPair.second;
            motor->update();
        }
        vTaskDelayUntil(&xLastWakeTime, static_cast<TickType_t>(1000.f / TASK_FREQ));
    }
}

uint8_t MotorManager::assignId() { return registedNum_++; }
