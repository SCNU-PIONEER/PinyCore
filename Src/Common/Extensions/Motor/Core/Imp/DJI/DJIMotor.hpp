#pragma once

#include "../../Base/MotorBase.hpp"

namespace PINYMOTOR::DJIMOTOR {
#pragma pack(push, 1)
struct Msg_s {
    int16_t cmd[4];
};
struct Feedback_s {
    uint16_t rawAng;
    int16_t rawRpm;
    int16_t current;
    uint8_t temperature;
};
#pragma pack(pop)
struct Status_s {
    float voltTxCodeSpan;
    float currTxCodeSpan;
    float currRxCodeSpan;
    float currRated;    // A
    float torqRated;    // Nm
    float voltMax;      // V
    float currMax;      // A
    float torqMax;      // Nm
    float torqConstant; // Nm/A

    Status_s &operator=(const Status_s &_other);
};

class DJIMotor : public QuadMotorBase {
    using Base = QuadMotorBase;

private:
    RxBus_s::CANRxBuf_s rxBuf_ = {}; // buffer for received data

    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len);
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s &_rxBuf);
    MotorTypeDef_e ctrl();

protected:
    Status_s status_;
    void registerRecvCallback();
    void cancelRecvCallback();
    void updateCtrlId();
    uint16_t ctrlId_ = 0xFFFF; // sendId - depends on work mode

public:
    DJIMotor(const char _name[16], InitConfig_s _config);
    ~DJIMotor() override;
    void overrideStats(const Status_s &_newStats);

    uint16_t canId() const; // QuadMotor's canId is fixed
    uint16_t masterId() const;

    uint16_t uid() final;
    MotorTypeDef_e update() final;

    QuadMotorGroup_s *findGroup();
};

} // namespace PINYMOTOR::DJIMOTOR
