#pragma once

#include "../../Base/MotorBase.hpp"

#include <memory>

namespace PINYMOTOR {
namespace DJI_ODMOTOR {
#pragma pack(push, 1)
struct Msg_s {
    int16_t cmd[3];
};

struct Feedback_s {
    uint16_t rawScale;
    int16_t rawTorq;
};
#pragma pack(pop)
struct Status_s {
    float voltTxCodeSpan;
    float torqRxCodeSpan;
    float currRated;    // A
    float torqRated;    // Nm
    float voltMax;      // V
    float currMax;      // A
    float torqMax;      // Nm
    float torqConstant; // Nm/A

    Status_s &operator=(const Status_s &_other);
};

class DJI_ODMotor : public TripMotorBase {
    using Base = TripMotorBase;

private:
    RxBus_s::CANRxBuf_s rxBuf_;

protected:
    Status_s status_;
    void registerRecvCallback();
    void cancelRecvCallback();
    void updateCtrlId();
    uint16_t ctrlId_ = 0xFFFF; // sendId - depends on work mode

public:
    DJI_ODMotor(const char _name[16], InitConfig_s _config);
    ~DJI_ODMotor() override;
    void overrideStats(const Status_s &_newStats);

    uint16_t canId() const; // TripMotor's canId is fixed
    uint16_t masterId() const;
    uint16_t uid() override final;
    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf,
                        uint8_t _len) override final;
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s &_rxBuf);
    MotorTypeDef_e ctrl() override final;
    TripMotorGroup_s *findGroup() const;
};
}
}
