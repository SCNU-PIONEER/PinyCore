#pragma once

#include "IMotor.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <unordered_map>
#include <cstring>
#include <functional>

namespace PINYMOTOR {

// 模板类，用于定义一拖一电机的基类
class MotorBase : public IMotor {
protected:
    uint32_t *pComHandle_;
    ComType_e comType_;
    WorkMode_e workMode_;
    GlobalState_e globalState_;
    uint8_t offsetId_;

    // 用户自定义回调函数
    std::function<void(const uint8_t *_rxBuffer)> userRecvCallback_;

    bool checkSend() const;

    bool isMutiple_ = false; // default is not quad encoder

    uint32_t lastSendTick = 0; // ms

public:
    MotorBase(const char _name[16], InitConfig_s _config);
    void
    regUserRecvCallback(std::function<void(const uint8_t *_rxBuf)> _callback);
};

/*******************************************************************/
struct QuadMotorGroup_s {
    IMotor *motor[4];
    uint32_t lastSendTick; // ms
    float minTxFreq;
    QuadMotorGroup_s();
    void showMotorInfo();
};
// 模板类，用于定义一拖四电机的基类
class QuadMotorBase : public MotorBase {
    using Base = MotorBase;
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
// 模板类，用于定义一拖三电机的基类
class TripMotorBase : public MotorBase {
    using Base = MotorBase;
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
