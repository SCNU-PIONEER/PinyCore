#pragma once

#include "IMotor.hpp"
#include "FreeRTOS.h"
#include "message_buffer.h"

#include <array>

namespace PINYMOTOR::MTMOTOR {

enum class WorkMode_e : MotorTypeDef_e {
    UNKNOWN,
    PDESVDES,
    CURR,
};

struct Status_s {
    uint16_t speedMax; //dps
    float currMax;     // A
    float torqMax;     // Nm
    float np;          // Nm
    float interRR;
    float kn; // Nm/A

    Status_s &operator=(const Status_s &_other);
};

class MTMotor : public IMotor {
    using ConvertFunc = void (MTMotor::*)(std::array<uint8_t, 8> &);

public:
    MTMotor(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);
    ~MTMotor() override;
    MotorTypeDef_e update() final;

    void overrideStats(const Status_s &_newStats);

    bool isEnable() const;
    void overrideReductionRatio(float _newReductionRatio) final;

protected:
    void registerRecvCallback(uint16_t _rxId);
    void updateCtrlMode();

    Status_s status_;
    WorkMode_e workMode_ = WorkMode_e::UNKNOWN;
    bool isSupportMode(WorkMode_e _mode) const;

private:
    MotorTypeDef_e ctrl();
    MotorTypeDef_e send(uint16_t _sendId, std::array<uint8_t, 8> _txBuf, uint8_t _len);

    MotorTypeDef_e parse(const uint8_t *_rxBuf);
    MotorTypeDef_e parseAbsPosCtrl(const uint8_t *_rxBuf);
    MotorTypeDef_e parseTorqCtrl(const uint8_t *_rxBuf);
    MotorTypeDef_e parseReadState2(const uint8_t *_rxBuf);

    void disable(std::array<uint8_t, 8> &_txBuf);
    void readState2(std::array<uint8_t, 8> &_txBuf);
    void absPosCtrl(std::array<uint8_t, 8> &_txBuf);
    void torqCtrl(std::array<uint8_t, 8> &_txBuf);

    ConvertFunc convert = &MTMotor::disable;

    MessageBufferHandle_t rxStream_;
    uint16_t ctrlId_ = 0XFFFF;
};

} // namespace PINYMOTOR::MTMOTOR
