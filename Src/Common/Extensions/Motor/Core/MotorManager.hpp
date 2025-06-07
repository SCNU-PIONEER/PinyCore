#pragma once

#include <stdint.h>
#include <unordered_map>

namespace PINYMOTOR {
class IMotor;
class MotorManager {
    using LoggerCallback =
            void(*)(const char *, const char *, const char *, ...);
private:
    inline static LoggerCallback activeLogger_;
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

    /* LOG */
    // 建议使用lambda适配LOG原型
    inline void registerLogger(LoggerCallback logger) {
        activeLogger_ = std::move(logger);
    }

    template<typename... Args>
    inline void log(const char* type, const char* color, const char* format, Args... args) {
        if (activeLogger_) {
            activeLogger_(type, color, format, args...);
        }
    }

private:
    MotorManager() = default;

    const float motorTaskFreq_ = 1000.f;

    std::unordered_map<uint8_t, IMotor *> motorList_;
};
}
