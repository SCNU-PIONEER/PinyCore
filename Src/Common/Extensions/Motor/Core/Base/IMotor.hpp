#pragma once

#include <cstdint>

#include "../Projdefs.hpp"

#include <queue>

#include <cstdarg>

namespace PINYMOTOR {
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

    Data_s &data();
    
    float RR() const;
    float span() const;
    float txFreq() const;

    void overrideReductionRatio(float _newReductionRatio);
    void overrideMeasureMax(float _newMeasureMax);
    void overrideMeasureMin(float _newMeasureMin);

    const char *getName() const;
    void log(const char *type, const char *color, const char *format, ...);

    // Mock functions for testing
    void overrideTxBaseId(uint16_t _newTxBaseId);
    void overrideRxBaseId(uint16_t _newRxBaseId);
};
}
