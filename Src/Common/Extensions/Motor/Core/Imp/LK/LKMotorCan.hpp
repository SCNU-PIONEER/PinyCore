#pragma once

#include "IMotor.hpp"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "LKMotorMsg.hpp"

namespace PINYMOTOR::LKMOTOR {

class LKMotorCAN : public IMotor {
    using TxBus = TxBus_s::CANTxBuf_s<8>;
    using ConvertFunc = void (LKMotorCAN::*)(TxBus &);

    static constexpr uint16_t TX_BASE_ID = 0x140;
    static constexpr uint16_t RX_BASE_ID = 0x140;

public:
    MotorTypeDef_e update() final;
    bool isEnable() const;
    ~LKMotorCAN() override;

    void overrideStats(const Status_s &_stats);

    MotorTypeDef_e readState1andError();
    MotorTypeDef_e cleanError();
    MotorTypeDef_e readState2();
    MotorTypeDef_e readState3();
    MotorTypeDef_e disable();
    MotorTypeDef_e enable();
    MotorTypeDef_e stop();

    MotorTypeDef_e readEncoder();
    MotorTypeDef_e setZeroPos();
    MotorTypeDef_e readMultiPos();
    MotorTypeDef_e clearPosCircle();
    MotorTypeDef_e readSinglePos();
    MotorTypeDef_e readCtrlCmd(const ParamID_e _id);
    MotorTypeDef_e writeCtrlCmd(const ParamID_e _id, const std::array<uint8_t, 6> _data);
    MotorTypeDef_e setPos(const float _angle);
    void switchCtrlMode(WorkMode_e _workMode);

    uint16_t canId() const;

protected:
    LKMotorCAN(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);

    void registerRecvCallback(uint16_t _rxId);
    void cancelRecvCallback(uint16_t _rxId);
    void updateCtrlMode();
    void overrideReductionRatio(float _newReductionRatio) override;

    MotorTypeDef_e txConvert(const uint8_t _cmdid);
    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len);

    Status_s status_;
    WorkMode_e workMode_ = WorkMode_e::UNKNOWN;
    bool isSupportMode(WorkMode_e _mode) const;

private:
    MotorTypeDef_e ctrl();
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf);
    MotorTypeDef_e parseState1andError(const uint8_t *_data);
    MotorTypeDef_e parseState2(const uint8_t *_data);
    MotorTypeDef_e parseCtrlCmd(const uint8_t *_data);
    MotorTypeDef_e parseEncoder(const uint8_t *_data);
    MotorTypeDef_e parsePosZero(const uint8_t *_data);
    MotorTypeDef_e parseMultiPos(const uint8_t *_data);
    MotorTypeDef_e parseSinglePos(const uint8_t *_data);
    MotorTypeDef_e parseSetPos(const uint8_t *_data);

    void convertVOLT(TxBus &_txBuf);
    void convertMIT(TxBus &_txBuf);
    void convertVDES(TxBus &_txBuf);
    void convertMULTIPDES(TxBus &_txBuf);
    void convertMULTIPDESVDES(TxBus &_txBuf);
    void convertSINGLEPDES(TxBus &_txBuf);
    void convertSINGLEPDESVDES(TxBus &_txBuf);
    void convertINCPDES(TxBus &_txBuf);
    void convertINCPDESVDES(TxBus &_txBuf);
    void convertDisable(TxBus &_txBuf);

    ConvertFunc convert = &LKMotorCAN::convertDisable;

    ErrorState_e errorState_;
    uint16_t ctrlId_ = 0xFFFF;
};

} // namespace PINYMOTOR::LKMOTOR