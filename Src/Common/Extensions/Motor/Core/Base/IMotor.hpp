#pragma once

#include <cstdint>
#include "FreeRTOS.h"
#include "queue.h"
#include "../Projdefs.hpp"
#include <cstdarg>
#include <memory>

namespace PINYMOTOR {
class IMotor {
protected:
    uint8_t id_; // start from 0 to 31, system auto assign
    uint16_t uid_;

    Model_s model_;
    Data_s data_;
    Cmd_s cmd_;

    uint32_t *pComHandle_;
    ComType_e comType_;
    WorkMode_e workMode_;
    GlobalState_e globalState_;
    uint8_t offsetId_;

    float txFreq_;
    float rxFreq_;             // TODO:
    uint32_t lastSendTick = 0; // ms
    uint32_t lastRecvTick = 0; // ms
    char name_[16] = "NULL";

    std::unique_ptr<PID> posPID_;
    std::unique_ptr<PID> velPID_;

    std::unique_ptr<PID> torqPID_; // only VOLT-CTRL motor will need this

    QueueHandle_t rxQueue_; // TODO: use a queue to store the received data

    QueueHandle_t cmdQueue_;
    CmdBus_s cmdBuf_;
    
    bool isMutiple_ = false; // default is not quad encoder

    bool checkSend();
    void calcRecvFreq();
    void parseCmd();

public:
    IMotor(const char _name[16], InitConfig_s _config);
    virtual ~IMotor() = default;

    virtual MotorTypeDef_e update() = 0;

    virtual uint16_t uid() = 0;
    uint8_t id() const;

    MotorTypeDef_e registerMotor();
    MotorTypeDef_e cancelMotor();

    MotorTypeDef_e cmd(MotorCmdType_e _cmd, float _pos, float _vel,
                       float _torq);
    MotorTypeDef_e cmd(MotorCmdType_e _cmd, float _cmdData);
    MotorTypeDef_e cmd(MotorCmdType_e _cmd);

    Data_s &data();

    float getCmdCurr();

    float txBaseId() const;
    float rxBaseId() const;

    float rr() const;
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

} // namespace PINYMOTOR
