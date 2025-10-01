#pragma once

#include <cstdint>
#include <unordered_map>

#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"

#define MOTOR_PARSE_EVENT_MAST 0xFFFFFFFF
namespace PINYMOTOR {
class IMotor;
class MotorManager {
public:
    MotorManager(const MotorManager &) = delete;
    MotorManager &operator=(const MotorManager &) = delete;

    static MotorManager *instance();

    void init();

    void parseMsg();

    void ctrlTask();

    void taskCreate();

    uint8_t assignId();

    // <uint16_t, IMotor *> -> <uid, motor>
    std::unordered_map<uint8_t, IMotor *> &motors() { return motorList_; }

private:
    MotorManager() = default;

    const float motorTaskFreq_ = 1000.f;

    std::unordered_map<uint8_t, IMotor *> motorList_;
    uint8_t registedNum_ = 0;
};
} // namespace PINYMOTOR
