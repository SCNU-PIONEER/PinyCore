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
public:
    MotorManager(const MotorManager &) = delete;
    MotorManager &operator=(const MotorManager &) = delete;

    static MotorManager *instance();

    void parseMsg();

    void task();

    uint8_t assignId();

    // <uint16_t, IMotor *> -> <uid, motor>
    std::unordered_map<uint8_t, IMotor *> &motors() { return motorList_; }

private:
    MotorManager() : Task("MotorTask", TaskPriority_e::HIGH2) {};

    const float motorTaskFreq_ = 1000.f;

    std::unordered_map<uint8_t, IMotor *> motorList_;
    uint8_t registedNum_ = 0;
};

} // namespace PINYMOTOR
