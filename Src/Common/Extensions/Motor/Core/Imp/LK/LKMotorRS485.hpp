#pragma once

#include "IMotor.hpp"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "Bsp_uart.hpp"
#include "LKMotorRS485Msg.hpp"

namespace PINYMOTOR::LKMOTOR {

struct Status_s {
    float powerMax;            // 峰值功率
    float torqueMax;           // 峰值扭矩
    int32_t speedMax;          // 峰值转矩
    float speedConstant;       // 转速常数
    float torqueConstant;      // 扭矩常数
    int16_t txCurrMax;         // 发送电流最大值
    float innerReductionRatio; // 内部减速比

    Status_s() = default;

    Status_s(float _powerMax, float _torqueMax, int32_t _speedMax, float _speedConstant, float _torqueConstant,
             int16_t _txCurrMax, float _innerReductionRatio)
            : powerMax(_powerMax)
            , torqueMax(_torqueMax)
            , speedMax(_speedMax)
            , speedConstant(_speedConstant)
            , torqueConstant(_torqueConstant)
            , txCurrMax(_txCurrMax)
            , innerReductionRatio(_innerReductionRatio) {};
};

class LKMotorRS485 : public IMotor {
public:
    MotorTypeDef_e update() final;
    bool isEnable() const;
    ~LKMotorRS485() override;

protected:
    LKMotorRS485(const char _name[16], InitConfig_s _config);

    void registerRecvCallback();
    void updateCtrlMode(const WorkMode_e _mode);
    void overrideReductionRatio(float _newReductionRatio) final
    {
        UNUSED(_newReductionRatio);
    }; // TODO: waiting for coding

    template <uint8_t len> MotorTypeDef_e send(const TransmitMsg_s<len> *_txBuf, uint8_t _len);
    MotorTypeDef_e txConvert(const uint8_t _cmdid);
    template <uint8_t len> requires(len > 0) MotorTypeDef_e txConvert(const uint8_t _cmdid, const uint8_t *_data);

    MotorTypeDef_e (LKMotorRS485::*ctrlCallback)();

    MotorTypeDef_e readState1andError();
    MotorTypeDef_e cleanError();
    MotorTypeDef_e readState2();
    MotorTypeDef_e readState3(); // MS motor can't read state3
    MotorTypeDef_e disable();
    MotorTypeDef_e enable();
    MotorTypeDef_e stop();
    MotorCmdType_e breakCtrl();    // TODO: waiting for coding
    MotorTypeDef_e openloopCtrl(); // only for MS motor
    MotorTypeDef_e torqCtrl();     //only for MS/MH/MG motor
    MotorTypeDef_e velCtrl();
    MotorTypeDef_e multiPosCtrl1();
    MotorTypeDef_e multiPosCtrl2();
    MotorTypeDef_e singlePosCtrl1();
    MotorTypeDef_e singlePosCtrl2();
    MotorTypeDef_e incrementalPosCtrl1();
    MotorTypeDef_e incrementalPosCtrl2();

    MotorTypeDef_e readCtrlCmd(const ParamID_e _id);
    // write to RAM, not flash, so that the change is effective immediately, but will be lost after power off
    MotorTypeDef_e writeCtrlCmd(const ParamID_e _id, const std::array<uint8_t, 6> _data);
    MotorTypeDef_e readEncoder();
    // WARN: write to flash, don't change the value frequently, otherwise it will shorten the life of flash
    MotorTypeDef_e setZeroPos();
    MotorTypeDef_e readMultiPos();
    MotorTypeDef_e clearPosCircle();
    MotorTypeDef_e readSinglePos();
    MotorTypeDef_e setPos(const float _angle);

    Status_s status_;

private:
    MotorTypeDef_e ctrl();

    MotorTypeDef_e parse();
    MotorTypeDef_e checkSum(const uint8_t *_data, const uint8_t _len);
    MotorTypeDef_e parseState1andError();
    MotorTypeDef_e parseState2();
    MotorTypeDef_e parseCtrlCmd();
    MotorTypeDef_e parseEncoder();
    MotorTypeDef_e parsePosZero();
    MotorTypeDef_e parseMultiPos();
    MotorTypeDef_e parseSinglePos();
    MotorTypeDef_e parseSetPos();

private:
    Uart uart_;
    uint8_t *txBuf_ = nullptr;
    uint8_t *rxBuf_ = nullptr;
    static constexpr uint8_t TXBUF_LEN = 20;
    static constexpr uint8_t RXBUF_LEN = 20;

    ErrorState_e errorState_;
};

} // namespace PINYMOTOR::LKMOTOR
