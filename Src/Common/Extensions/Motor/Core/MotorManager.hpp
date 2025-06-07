#pragma once

#include <stdint.h>
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

    void ctrlTask();

    void taskCreate();

    // <uint16_t, IMotor *> -> <uid, motor>
    inline std::unordered_map<uint8_t, IMotor *> &motors()
    {
        return motorList_;
    }

private:
    MotorManager() = default;

    const float motorTaskFreq_ = 1000.f;

    std::unordered_map<uint8_t, IMotor *> motorList_;
};
}
