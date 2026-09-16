#pragma once
#include "../Base/MotorBase.hpp"
#include "Bsp_can.hpp"
#include "StmLog.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "MotorCommonMacros.hpp"
#include <cstdint>
#include "LKMotorMsg.hpp"

namespace PINYMOTOR::LKMOTOR {

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

    WorkMode_e workMode_ = WorkMode_e::UNKNOWN;
    bool isSupportMode(WorkMode_e _mode) const;

    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len);
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf);
    MotorTypeDef_e ctrl();

public:
    LKMotor(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);
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
