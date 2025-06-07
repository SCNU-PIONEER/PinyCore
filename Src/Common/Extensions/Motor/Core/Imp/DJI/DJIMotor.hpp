#pragma once

#include "../../Base/MotorBase.hpp"

#include <memory>

namespace PINYMOTOR {

#pragma pack(push, 1)
struct DJIMotorMsg_s {
    int16_t cmd[4];
};
struct DJIMotorFeedback_s {
    uint16_t rawScale;
    int16_t rawRpm;
    int16_t current;
    uint8_t temperature;
};
#pragma pack(pop)
struct DJIMotorStats_s {
    float voltTxCodeSpan;
    float currTxCodeSpan;
    float currRxCodeSpan;
    float currRated;    // A
    float torqRated;    // Nm
    float voltMax;      // V
    float currMax;      // A
    float torqMax;      // Nm
    float torqConstant; // Nm/A

    DJIMotorStats_s& operator=(const DJIMotorStats_s& _other);
};

class DJIMotor : public QuadMotorBase {
    using Base = QuadMotorBase;
private:
    struct CmdInternal_s;
    std::unique_ptr<CmdInternal_s> cmd_;
protected:
    DJIMotorStats_s stats_;
    void registerRecvCallback();
    void cancelRecvCallback();
    uint16_t ctrlId_ = 0x00; // sendId - depends on work mode

public:
    DJIMotor(const char _name[16], InitConfig_s _config);
    ~DJIMotor() override;
    void overrideStats(const DJIMotorStats_s &_newStats);
    
    uint16_t canId() const; // QuadMotor's canId is fixed
    uint16_t masterId() const;

    MotorTypeDef_e cmd(MotorCmdType_e _cmd, float _cmdData) override final;
    MotorTypeDef_e cmd(MotorCmdType_e _cmd) override final;
    uint16_t uid() override final;
    MotorTypeDef_e send (uint8_t *_txBuf, uint8_t _len) override final;
    MotorTypeDef_e parse(const uint8_t *_rxBuf) override final;
    MotorTypeDef_e ctrl() override final;

    QuadMotorGroup_s *findGroup() const;
};

struct DJIMotor::CmdInternal_s {
    bool SW;
    bool prevSW;
    struct {
        float torq;
        float volt;
    };
    void clear();
    void updateSW(bool _sw);
};
}
