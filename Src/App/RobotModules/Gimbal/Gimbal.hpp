#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"
#include "TopicRouter.hpp"

#include "IMotor.hpp"
#include "MotorCommonMacros.hpp"
#include "INS.hpp"

#include "Bsp_can.hpp"
#include "PidBasic.hpp"


#include "sdkconfig.h"

extern bool constloadedGimbal;

namespace GIMBAL {
enum class FSMState_e : uint8_t { STOP = 0, LAUNCH, RUN};

struct MotorCtrl_s {
    PositionalPid *posDualPid;
    PositionalPid *velDualPid;//双环PID
    PositionalPid *velSingPid;//单环PID
    float targetVel;
    float torq;
};
}

// extern canHandle HCAN2;
// class ChassisTxPacket : public COMM::TxPacket<COMM::ChassisCtrlPacket> {
// public:
//     ChassisTxPacket(float _txFreq)
//             : COMM::TxPacket<COMM::ChassisCtrlPacket>(_txFreq)
//     {
//     }
//     void transmit(uint8_t *_buf, uint16_t _len) final
//     {
//         Can::instance().transmitData(&HCAN2, uid(), _buf, _len);
//     }
// };

// class DialTxPacket : public COMM::TxPacket<COMM::DialCtrlPacket> {
// public:
//     DialTxPacket(float _txFreq) : COMM::TxPacket<COMM::DialCtrlPacket>(_txFreq)
//     {
//     }
//     void transmit(uint8_t *_buf, uint16_t _len) final
//     {
//         Can::instance().transmitData(&HCAN2, uid(), _buf, _len);
//     }
// };
class Gimbal {
public:
//    some constant
    Gimbal()
            : insSub_(new Subscriber<INS_SYS::INSData_s>(
                      4, &TopicRouter::instance().insTopic, &insData_))
    {
    }
    virtual ~Gimbal() {}

    virtual void stop() = 0;
    virtual void enter() = 0;
    virtual void updateSelf() = 0;


    void update(void *_param)
    {
        if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &gmsg, 0) ==
            pdTRUE) {
        };

        this->insSub_->receive();


#if APP_USE_COMM
        // if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg, 0) ==
        //     pdTRUE) {
        // };
        // if (xQueueReceive((((MsgBus_s *)_param)->dialQueue), &dialMsg, 0) ==
        //     pdTRUE) {
        // };

        // if (!isTurn && msg.isTurn)
        //     isTurn = true;

        // COMM::ChassisCtrlPacket::ProtoData_s chassisData = {};
        // chassisData.vx = cmsg.vx;
        // chassisData.vy = cmsg.vy;
        // chassisData.deltaYaw = this->deltaYaw_;
        // chassisData.state = cmsg.state;
        // chassisData.isGimbalLaunched = isLaunched;
        // chassisData.isTurning = isTurn;
        // chassisCtrlPacket.setData() = chassisData;

        // COMM::DialCtrlPacket::ProtoData_s dialData = {};
        // dialData.isFire = dialMsg.isFire;
        // dialData.state = dialMsg.state;
        // dialData.isGimbalLaunched = isLaunched;
        // dialCtrlPacket.setData() = dialData;
#else
        // deltaYawMsg_.deltaYaw = deltaYaw_;
        // deltaYawPub_.publish();
#endif
        updateSelf();
    }

    const INS_SYS::INSData_s &ins() const { return insData_; }

    GimbalMsg_s gmsg = {};
    bool yawLaunched = false;
    bool trigLaunched = false;
    bool alreadyLaunched = false;
    bool isLaunched = false;

    static constexpr float angyRange = 11.5f;//云台0~11.5度
    static constexpr float looptRange = -30.f;//扳机向下圈数30
    static constexpr float angAcc = 2.f;//云台分部转动每次最大幅度
    static constexpr float loopAcc = 1.f;//扳机分部转动每次最大圈圈数
    static constexpr float ratioAngle2Loop = std::fabsf((28.175f / angyRange));//云台转1度对应电机转多少圈
    static constexpr float ratio2006Gear = 36.f;//减速箱减速比

    static constexpr float YAW_LAUNCH_ANG_DEADBAND = 0.1f;
    static constexpr float TRIG_LAUNCH_ANG_DEADBAND = 0.005f;

    static constexpr float constangYaw[5] = {7.999f, 3.99f, 1.999f, 0.f, 0.f};
    static constexpr float constangTrig[5] = {0.f, 0.f, 0.f, 0.f, 0.f};//-6.26f, -6.38f, -5.49f, -5.23f, 0.f
protected:

    StateFactory<GIMBAL::FSMState_e> stateFactory_;
    INS_SYS::INSData_s insData_ = {};
    Subscriber<INS_SYS::INSData_s> *insSub_;

#if APP_USE_COMM
    // ChassisTxPacket chassisCtrlPacket;
    // DialTxPacket dialCtrlPacket;
#else
    DeltaYawMsg_s deltaYawMsg_{};
    Publisher<DeltaYawMsg_s> deltaYawPub_{
        &TopicRouter::instance().deltaYawTopic, &deltaYawMsg_
    };
#endif

private:
};
