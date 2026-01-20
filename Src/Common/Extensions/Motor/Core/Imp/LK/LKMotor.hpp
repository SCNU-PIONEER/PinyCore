#pragma once
#include "../Base/MotorBase.hpp"
#include "Bsp_can.hpp"
#include "StmLog.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "MotorCommonMacros.hpp"
#include <cstdint>
namespace PINYMOTOR::LKMOTOR {

#pragma pack(push, 1)
struct Feedback_s {
    uint8_t cmd;
    uint8_t temperature;
    int16_t current;
    int16_t speed;
    uint16_t angle;
};

#pragma pack(pop)
struct Status_s {
    float txcurrentDataMax; // 发送电流最大编码值
    float rxcurrentDataMax; // 接收电流最大编码值
    float torqueMax;        // 扭矩最大值
    float txcurrentMax;     // 发送电流最大值
    float rxcurrentMax;     // 接收电流最大值
    float torqueConstant;   // 转矩常数


    Status_s() = default;

    Status_s(float _txcurrDataMax, float _rxcurrDataMax, float _tqMax, float _txcurrMax, float _rxcurrMax,
             float _tqConst)
            : txcurrentDataMax(_txcurrDataMax)
            , rxcurrentDataMax(_rxcurrDataMax)
            , torqueMax(_tqMax)
            , txcurrentMax(_txcurrMax)
            , rxcurrentMax(_rxcurrMax)
            , torqueConstant(_tqConst)

    {
    }
};

class LKMotor : public QuadMotorBase {
    using Base = QuadMotorBase;
    using TxBus = TxBus_s::CANTxBuf_s<8>;

    static constexpr uint16_t TX_BASE_ID = 0x280;
    static constexpr uint16_t RX_BASE_ID = 0x140;

protected:
    uint16_t currentTxId_;
    uint8_t motorIndex_;
    RxBus_s::CANRxBuf_s<8> rxBuf_ = {};
    Status_s status_;

    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len);
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf);
    MotorTypeDef_e ctrl();

    virtual MotorTypeDef_e checkBaseConfig() = 0;
    virtual void initModelParams() = 0;

public:
    LKMotor(const char _name[16], InitConfig_s _config);
    ~LKMotor() override;

    void overrideStats(const Status_s &_newStats);
    void overrideReductionRatio(float _newReductionRatio) override;
    void registerRecvCallback();
    void cancelRecvCallback();
    void updateTxId();
    uint16_t canId() const;
    uint16_t masterId() const;
    MotorTypeDef_e update() override;
};

} // namespace PINYMOTOR::LKMOTOR
