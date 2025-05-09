#pragma once

#include <cstdint>

#include "../Projdefs.hpp"

#include <queue>

#include <cstdarg>

#include "PinyCore.hpp"

namespace PINYMOTOR {
class MotorManager;
// 类型擦除 管理异构CRTP
class IMotor {
protected:
    Model_s model_;
    Data_s data_;
    float txFreq_;
    float rxFreq_; // TODO:
    char name_[16] = "NULL";

    std::queue<MotorErrorCode_e> errQueue_;
public:
    virtual ~IMotor() = default;
    virtual MotorTypeDef_e send(uint8_t *_txBuffer, uint8_t _txLen) = 0;
    virtual MotorTypeDef_e parse(const uint8_t *_rxBuffer) = 0;
    virtual MotorTypeDef_e ctrl() = 0;

    virtual MotorTypeDef_e cmd(MotorCmdType_e _cmd, float _cmdData) = 0;
    virtual MotorTypeDef_e cmd(MotorCmdType_e _cmd) = 0;

    virtual uint16_t uid() = 0;

    MotorTypeDef_e registerMotor();
    MotorTypeDef_e cancelMotor();

    inline Data_s &data() { return data_; }
    // 出轴减速比
    inline float RR(){ return model_.reductionRatio; }
    // 角度编码量程
    inline float span()
    {
        return static_cast<float>(model_.measureMax - model_.measureMin);
    }

    inline float txFreq() const { return txFreq_; }

    inline const char *getName() const { return name_; }

    void log(const char *type, const char *color, const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        PinyCore::instance()->log(type, color, format, args);
        va_end(args);
    }
};
}
