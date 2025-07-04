#pragma once

#include "../../Base/MotorBase.hpp"

#include <memory>

namespace PINYMOTOR {

#pragma pack(push, 1)
struct DJI_ODMotorMsg_s {
    int16_t cmd[3];
};

struct DJI_ODMotorFeedback_s {
    uint16_t rawScale;
    int16_t rawTorq;
};
#pragma pack(pop)
struct DJI_ODMotorStats_s {
    float voltTxCodeSpan;
    float torqRxCodeSpan;
    float currRated;    // A
    float torqRated;    // Nm
    float voltMax;      // V
    float currMax;      // A
    float torqMax;      // Nm
    float torqConstant; // Nm/A

    DJI_ODMotorStats_s &operator=(const DJI_ODMotorStats_s &_other);
};

class DJI_ODMotor : public TripMotorBase {
    using Base = TripMotorBase;

protected:
    DJI_ODMotorStats_s stats_;
    void registerRecvCallback();
    void cancelRecvCallback();
    void updateCtrlId();
    uint16_t ctrlId_ = 0xFFFF; // sendId - depends on work mode

public:
    DJI_ODMotor(const char _name[16], InitConfig_s _config);
    ~DJI_ODMotor() override;
    void overrideStats(const DJI_ODMotorStats_s &_newStats);

    uint16_t canId() const; // TripMotor's canId is fixed
    uint16_t masterId() const;
    uint16_t uid() override final;
    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf,
                        uint8_t _len) override final;
    MotorTypeDef_e parse(const uint8_t *_rxBuf) override final;
    MotorTypeDef_e ctrl() override final;
    TripMotorGroup_s *findGroup() const;
};
};
