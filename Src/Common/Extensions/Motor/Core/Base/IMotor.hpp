#pragma once

#include <cstdint>
#include "FreeRTOS.h"
#include "queue.h"
#include "../Projdefs.hpp"
#include <cstdarg>
#include <memory>
#include "StmLog.hpp"

namespace PINYMOTOR {
class IMotor {
protected:
    uint8_t id_; // start from 0 to 31, system auto assign
    uint16_t uid_;

    Model_s model_;
    Data_s data_;
    Cmd_s cmd_;

    float txFreq_;
    float rxFreq_; // TODO:
    char name_[16] = "NULL";

    std::unique_ptr<PID> posPID_;
    std::unique_ptr<PID> velPID_;

    std::unique_ptr<PID> torqPID_; // only VOLT-CTRL motor will need this

    MotorCmdType_e curCmdType_ = MotorCmdType_e::OFF;

    QueueHandle_t rxQueue_; // TODO: use a queue to store the received data

public:
    IMotor();
    virtual ~IMotor() = default;
    virtual MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuffer,
                                uint8_t _txLen) = 0;
    virtual MotorTypeDef_e ctrl() = 0;

    virtual uint16_t uid() = 0;
    uint8_t id() const;

    MotorTypeDef_e registerMotor();
    MotorTypeDef_e cancelMotor();

    MotorTypeDef_e cmd(MotorCmdType_e _cmd, float _cmdData);
    MotorTypeDef_e cmd(MotorCmdType_e _cmd);

    Data_s &data();

    Cmd_s &cmd();

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
};

}
