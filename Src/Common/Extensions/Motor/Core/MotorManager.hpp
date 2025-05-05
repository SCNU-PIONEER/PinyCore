#pragma once

#include "../Projdefs.hpp"

#include <unordered_map>

namespace PINYMOTOR {
class IMotor;
class MotorManager {
public:
    MotorManager(const MotorManager &) = delete;
    MotorManager &operator=(const MotorManager &) = delete;

    inline static MotorManager* instance() {
        static MotorManager instance_;
        return &instance_;
    }

    void task();

    // <uint16_t, IMotor *> -> <uid, motor>
    inline std::unordered_map<uint16_t, IMotor *> &motors() { return motorList_; }

private:
    MotorManager() = default;

    const float motorTaskFreq_ = 1000.f;

    std::unordered_map<uint16_t, IMotor *> motorList_;
};
}
