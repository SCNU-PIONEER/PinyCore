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

    DJI_ODMotorStats_s& operator=(const DJI_ODMotorStats_s& _other);
};

class DJI_ODMotor : public TripMotorBase {
    using Base = TripMotorBase;
private:
    struct CmdInternal_s;
    std::unique_ptr<CmdInternal_s> cmd_;
protected:
    DJI_ODMotorStats_s stats_;
    void registerRecvCallback();
    void cancelRecvCallback();
    uint16_t ctrlId_ = 0x00; // sendId - depends on work mode

public:
    DJI_ODMotor(const char _name[16], InitConfig_s _config);
    void overrideStats(const DJI_ODMotorStats_s& _newStats);
    MotorTypeDef_e cmd(MotorCmdType_e _cmd, float _cmdData) override final;
    MotorTypeDef_e cmd(MotorCmdType_e _cmd) override final;
    uint16_t canId() const; // TripMotor's canId is fixed
    uint16_t masterId() const;
    uint16_t uid() override final;
    MotorTypeDef_e send(uint8_t *_txBuf, uint8_t _len) override final;
    MotorTypeDef_e parse(const uint8_t *_rxBuf) override final;
    MotorTypeDef_e ctrl() override final;
    TripMotorGroup_s *findGroup() const;
};
struct DJI_ODMotor::CmdInternal_s {
    bool SW;
    bool prevSW;
    struct {
        float volt;
    };
    void clear();
    void updateSW(bool _sw);
};
};
