#pragma once

#include "IMotor.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <vector>
#include <unordered_map>
#include <cstring>

namespace PINYMOTOR {
struct QuadMotorGroup_s {
    IMotor *motor[4];
    TxBus_s::CANTxBuf_s<8> txBuf;
    uint8_t refLoadedCode;
    uint8_t curLoadedCode;
    uint32_t lastSendTick; // ms
    float minTxFreq;
    QuadMotorGroup_s();
    void showMotorInfo();
};
class QuadMotorBase : public IMotor {
    struct GroupBus_s {
        uint32_t *handle;
        std::unordered_map<uint16_t, QuadMotorGroup_s *> groupMap;
    };
    using Base = IMotor;
    using QuadMotors = std::vector<GroupBus_s>;

protected:
    QuadMotors &getMotorMap();
    void updateMotorMap();
    void removeMotorFromMap();

    QuadMotorGroup_s *group_ = nullptr;

public:
    QuadMotorBase(const char _name[16], InitConfig_s _config);
    uint16_t getGroupId() const;
    uint8_t getPosInGroup() const;
    bool checkGroupSend(QuadMotorGroup_s *_group);
};

/*******************************************************************/
struct TripMotorGroup_s {
    IMotor *motor[3];
    uint8_t txBuf[8];
    uint8_t refLoadedCode;
    uint8_t curLoadedCode;
    uint32_t lastSendTick; // ms
    float minTxFreq;
    TripMotorGroup_s();
    void showMotorInfo();
};
class TripMotorBase : public IMotor {
    struct GroupBus_s {
        uint32_t *handle;
        std::unordered_map<uint16_t, TripMotorGroup_s *> groupMap;
    };
    using Base = IMotor;
    using TripMotors = std::vector<GroupBus_s>;

protected:
    TripMotors &getMotorMap();
    void updateMotorMap();
    void removeMotorFromMap();

    TripMotorGroup_s *group_ = nullptr;

public:
    TripMotorBase(const char _name[16], InitConfig_s _config);
    uint16_t getGroupId() const;
    uint8_t getPosInGroup() const;
    bool checkGroupSend(TripMotorGroup_s *_group);
};

} // namespace PINYMOTOR
