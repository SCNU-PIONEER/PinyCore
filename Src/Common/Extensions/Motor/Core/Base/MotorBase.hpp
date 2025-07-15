#pragma once

#include "IMotor.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <unordered_map>
#include <cstring>

namespace PINYMOTOR {
struct QuadMotorGroup_s {
    IMotor *motor[4];
    uint32_t lastSendTick; // ms
    float minTxFreq;
    QuadMotorGroup_s();
    void showMotorInfo();
};
class QuadMotorBase : public IMotor {
    using Base = IMotor;
    using QuadMotors = std::vector<std::pair<
            uint32_t *, std::unordered_map<uint16_t, QuadMotorGroup_s *> > >;

private:
    // vector < pair(pComHandle_, <canId, 4 motors>) >
    static QuadMotors motorMap_;

protected:
    QuadMotors &getMotorMap() const;
    void updateMotorMap();
    void removeMotorFromMap();

public:
    QuadMotorBase(const char _name[16], InitConfig_s _config);
    uint16_t getGroupId() const;
    uint8_t getPosInGroup() const;
    bool checkGroupSend(QuadMotorGroup_s *_group);
};

/*******************************************************************/
struct TripMotorGroup_s {
    IMotor *motor[3];
    uint32_t lastSendTick; // ms
    float minTxFreq;
    TripMotorGroup_s();
    void showMotorInfo();
};
class TripMotorBase : public IMotor {
    using Base = IMotor;
    using TripMotors = std::vector<std::pair<
            uint32_t *, std::unordered_map<uint16_t, TripMotorGroup_s *> > >;

private:
    // vector < pair(pComHandle_, <canId, 3 motors>) >
    static TripMotors motorMap_;

protected:
    TripMotors &getMotorMap() const;
    void updateMotorMap();
    void removeMotorFromMap();

public:
    TripMotorBase(const char _name[16], InitConfig_s _config);
    uint16_t getGroupId() const;
    uint8_t getPosInGroup() const;
    bool checkGroupSend(TripMotorGroup_s *_group);
};
}
