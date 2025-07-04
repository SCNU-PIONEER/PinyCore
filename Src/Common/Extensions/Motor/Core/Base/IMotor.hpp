#pragma once

#include <cstdint>

#include "../Projdefs.hpp"

#include <queue>

#include <cstdarg>

#include "stm_log.hpp"

namespace PINYMOTOR {
class IMotor {
protected:
    Model_s model_;
    Data_s data_;
    Cmd_s cmd_;

    float txFreq_;
    float rxFreq_; // TODO:
    char name_[16] = "NULL";

    PID *posPID_;
    PID *velPID_;

    PID *torqPID_; // only VOLT-CTRL motor will need this

    MotorCmdType_e curCmdType_ = MotorCmdType_e::OFF;

    std::queue<MotorErrorCode_e> errQueue_;

public:
    virtual ~IMotor() = default;
    virtual MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuffer,
                                uint8_t _txLen) = 0;
    virtual MotorTypeDef_e parse(const uint8_t *_rxBuffer) = 0;
    virtual MotorTypeDef_e ctrl() = 0;

    virtual uint16_t uid() = 0;

    MotorTypeDef_e registerMotor();
    MotorTypeDef_e cancelMotor();

    MotorTypeDef_e cmd(MotorCmdType_e _cmd, float _cmdData);
    MotorTypeDef_e cmd(MotorCmdType_e _cmd);

    Data_s &data();

    float txBaseId() const;
    float rxBaseId() const;

    float RR() const;
    float measureMax() const;
    float measureMin() const;
    float span() const;
    float txFreq() const;
    float rxFreq() const;

    void overrideReductionRatio(float _newReductionRatio);
    void overrideMeasureMax(float _newMeasureMax);
    void overrideMeasureMin(float _newMeasureMin);

    const char *getName() const;

    template <typename... Args>
    void log(std::string_view _type, const char *color, const char *_format,
             Args &&...args);
};

template <typename... Args>
void IMotor::log(std::string_view _type, const char *color, const char *_format,
                 Args &&...args)
{
    LOG::info(_type, _format, std::forward<Args>(args)...,
              std::source_location::current());
}
}
