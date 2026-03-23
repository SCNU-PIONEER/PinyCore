#pragma once

#include <cstdint>
#include <unordered_map>

#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "Task.hpp"

namespace PINYMOTOR {

class IMotor;
class MotorManager : public Task<MotorManager, 512> {
    static constexpr float TASK_FREQ = 1000.f;
    friend IMotor;

public:
    MotorManager(const MotorManager &) = delete;
    MotorManager &operator=(const MotorManager &) = delete;

    static MotorManager *instance();

    void task();

private:
    MotorManager() : Task("MotorTask", TaskPriority_e::HIGH2) {};

    // <uint16_t, IMotor *> -> <uid, motor>
    std::unordered_map<uint8_t, IMotor *> &motors() { return motorList_; }
    uint8_t assignId();

    std::unordered_map<uint8_t, IMotor *> motorList_;
    uint8_t registedNum_ = 0;
};

} // namespace PINYMOTOR
