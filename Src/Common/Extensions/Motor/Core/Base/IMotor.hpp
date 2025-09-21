#pragma once

#include <cstdint>
#include "FreeRTOS.h"
#include "queue.h"
#include "../Projdefs.hpp"
#include <cstdarg>

namespace PINYMOTOR {
class IMotor {
private:
    MotorTypeDef_e cmdProto(CmdBus_s &_cmd);

protected:
    Data_s data_;
    Cmd_s cmd_;
    struct {
        uint16_t uid; // start from 0 to 31, system auto assign
        uint8_t offsetId;
        char name[16] = "NULL";
        uint32_t *pComHandle;
        Model_s model;
        ComType_e comType;
        WorkMode_e workMode;
        bool isReverse = false;
        bool isMutiple = false; // default is not quad encoder
    } regInfo_;                 // registration info

    struct {
        QueueHandle_t rxQueue;
        CmdBus_s cmdBuf;
        float txFreq;
        float rxFreq;
        uint16_t recvCnt;
        uint32_t lastSendTick = 0; // ms
        uint32_t lastRecvTick = 0; // ms
    } AUX_;                        // AUX info

    PID *posPID_;
    PID *velPID_;
    PID *torqPID_; // only VOLT-CTRL motor will need this

    bool checkSend();
    void calcRecvFreq();
    void parseCmd();

public:
    IMotor(const char _name[16], InitConfig_s _config);
    virtual ~IMotor() = default;

    virtual MotorTypeDef_e update() = 0;

    GlobalState_e globalState;

    uint16_t uid() const;

    MotorTypeDef_e registerMotor();
    MotorTypeDef_e cancelMotor();

    MotorTypeDef_e cmd(MotorCmdType_e _type);

    MotorTypeDef_e cmdPos(float _pos, float _velMax = -1.f, float _posMin = 0.f,
                          float _posMax = 0.f);

    MotorTypeDef_e cmdVel(float _vel, float _velMax = -1.f);

    MotorTypeDef_e cmdTorq(float _torq);

    MotorTypeDef_e cmdElec(float _elec);

    MotorTypeDef_e cmdPosVel(float _pos, float _vel, float _velMax = -1.f,
                             float _posMin = 0.f, float _posMax = 0.f);

    MotorTypeDef_e cmdMIT(float _pos, float _vel, float _torq,
                          float _velMax = -1.f, float _posMin = 0.f,
                          float _posMax = 0.f);

    void clampVel(float _velMax);
    void clampPos(float _posMin, float _posMax);
    void disableClampPos();

    const Data_s &data() const;

    void setZeroAng();
    void setZeroAng(float _zeroAng);

    float getCmdCurr();

    float txBaseId() const;
    float rxBaseId() const;

    float rr() const;
    float measureMax() const;
    float measureMin() const;
    float span() const;
    float txFreq() const;
    float rxFreq() const;

    float ang() const;
    float center() const;
    float pos() const;
    float posNorm() const;
    float vel() const;
    float torq() const;

    // Don't call it multiple times!
    virtual void overrideReductionRatio(float _newReductionRatio);

    void overrideMeasureMax(float _newMeasureMax);
    void overrideMeasureMin(float _newMeasureMin);

    const char *getName() const;
};

} // namespace PINYMOTOR
