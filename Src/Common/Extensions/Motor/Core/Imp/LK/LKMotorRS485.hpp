#pragma once

#include "IMotor.hpp"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "Bsp_uart.hpp"
#include "LKMotorMsg.hpp"

namespace PINYMOTOR::LKMOTOR {


class LKMotorRS485 : public IMotor {
public:
    MotorTypeDef_e update() final;
    bool isEnable() const;
    ~LKMotorRS485() override;

    MotorTypeDef_e readState1andError();
    MotorTypeDef_e cleanError();
    MotorTypeDef_e readState2();
    MotorTypeDef_e readState3(); // MS motor can't read state3
    MotorTypeDef_e disable();
    MotorTypeDef_e enable();
    MotorTypeDef_e stop();
    [[deprecated("TBD")]] MotorCmdType_e breakCtrl();
    MotorTypeDef_e openloopCtrl(); // only for MS motor
    MotorTypeDef_e torqCtrl();     //only for MS/MH/MG motor
    MotorTypeDef_e velCtrl();
    MotorTypeDef_e multiPosCtrl1();
    MotorTypeDef_e multiPosCtrl2();
    MotorTypeDef_e singlePosCtrl1();
    MotorTypeDef_e singlePosCtrl2();
    MotorTypeDef_e incrementalPosCtrl1();
    MotorTypeDef_e incrementalPosCtrl2();
    [[deprecated("80系: motor version V3.0, Hardware version V2.4, Firmware version V2.36 固件还不支持")]] MotorTypeDef_e
    readCtrlCmd(const ParamID_e _id);
    [[deprecated("80系: motor version V3.0, Hardware version V2.4, Firmware version V2.36 固件还不支持, write to RAM, \
            not flash, so that the change is effective immediately, but will be lost after power off")]] MotorTypeDef_e
    writeCtrlCmd(const ParamID_e _id, const std::array<uint8_t, 6> _data);
    MotorTypeDef_e readEncoder();
    /*
     * WARN: write to flash, don't change the value frequently, otherwise it will shorten the life of flash
     * reset pos after power restarting
     */
    MotorTypeDef_e setZeroPos();
    MotorTypeDef_e readMultiPos();
    MotorTypeDef_e clearPosCircle();
    MotorTypeDef_e readSinglePos();
    MotorTypeDef_e setPos(const float _angle);

protected:
    LKMotorRS485(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);

    void registerRecvCallback();
    void updateCtrlMode(const WorkMode_e _mode);
    [[deprecated("TBD")]] void overrideReductionRatio(float _newReductionRatio) final { UNUSED(_newReductionRatio); };

    template <uint8_t len> MotorTypeDef_e send(const TransmitMsg_s<len> *_txBuf, uint8_t _len);
    MotorTypeDef_e txConvert(const uint8_t _cmdid);
    template <uint8_t len> requires(len > 0) MotorTypeDef_e txConvert(const uint8_t _cmdid, const uint8_t *_data);

    MotorTypeDef_e (LKMotorRS485::*ctrlCallback)();

    Status_s status_;
    WorkMode_e workMode_ = WorkMode_e::UNKNOWN;
    bool isSupportMode(WorkMode_e _mode) const;

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
