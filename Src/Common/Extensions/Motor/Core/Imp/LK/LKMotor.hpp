#pragma once
#include "../Base/MotorBase.hpp"
#include "Bsp_can.hpp"
#include "StmLog.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "MotorCommonMacros.hpp"

#ifndef MOTOR_DEF
#define MOTOR_DEF
#define MOTOR_OK    0
#define MOTOR_ERROR 1
#endif

namespace PINYMOTOR::LKMOTOR {

#pragma pack(push, 1)
struct Feedback_s {
    uint8_t cmd;         // 命令字节
    uint8_t temperature; // 电机温度
    int16_t current;     // 转矩电流编码值
    int16_t speed;       // 转子角速度
    uint16_t angle;      // 编码器位置
};

struct Transmit_s {
    int16_t data[4];
    bool sendFlag;
};

struct Status_s {
    float torqueDataMax;  // 扭矩最大编码值
    float torqueMax;      // 扭矩最大值
    float currentDataMax; // 电流最大编码值
    float currentMax;     // 电流最大值
    float torqueConstant; // 转矩常数
    float reductionRatio; // 减速比
    float encoderSpan;    // 编码器范围
    uint16_t txCurrId;    // 电流/扭矩控制发送ID
    uint16_t rxBaseId;    // 接收基础ID

    Status_s() = default;

    Status_s(float tqDataMax, float tqMax, float currDataMax, float currMax, float tqConst, float rr, float encSpan,
             uint16_t txCurrId, uint16_t rxBase)
            : torqueDataMax(tqDataMax)
            , torqueMax(tqMax)
            , currentDataMax(currDataMax)
            , currentMax(currMax)
            , torqueConstant(tqConst)
            , reductionRatio(rr)
            , encoderSpan(encSpan)
            , txCurrId(txCurrId)
            , rxBaseId(rxBase)
    {
    }
};
#pragma pack(pop)

class LKMotor : public QuadMotorBase {
    using Base = QuadMotorBase;

    using ConvertFunc = void (LKMotor::*)();

protected:
    uint16_t currentTxId_;
    uint8_t motorIndex_;
    RxBus_s::CANRxBuf_s<8> rxBuf_ = {};
    Transmit_s txBuf_ = {};
    Status_s status_;
    QueueHandle_t cmdQueue_;

    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len);
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf);
    MotorTypeDef_e ctrl();
    void updateTxId();

    virtual MotorTypeDef_e checkBaseConfig() = 0;
    virtual void initModelParams() = 0;

public:
    LKMotor(const char _name[16], InitConfig_s _config);
    ~LKMotor() override;

    MotorTypeDef_e init();

    void overrideStats(const Status_s &_newStats);
    void overrideReductionRatio(float _newReductionRatio) override;
    uint16_t canId() const;
    uint16_t masterId() const;
    uint16_t uid() const;
    MotorTypeDef_e update() override;
    void mountToBroadcastFrame(Transmit_s &_broadcastFrame);
    void registerRecvCallback();
    void cancelRecvCallback();
};

} // namespace PINYMOTOR::LKMOTOR