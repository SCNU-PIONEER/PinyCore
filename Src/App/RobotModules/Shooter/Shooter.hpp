#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"
#include "TopicRouter.hpp"

#include "IMotor.hpp"
#include "MotorCommonMacros.hpp"
#include "INS.hpp"

#include "TxPacket.hpp"
#include "PacketImpl.hpp"
#include "Bsp_can.hpp"
#include "PidBasic.hpp"
#include "sdkconfig.h"

#include "Bsp_pwm.hpp"

#define GPIO_SCREW_Pin GPIO_PIN_7
#define GPIO_SCREW_GPIO_Port GPIOE


namespace SHOOTER {
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

class Shooter {
public:
//    some constant
    Shooter()
            : insSub_(new Subscriber<INS_SYS::INSData_s>(
                      4, &TopicRouter::instance().insTopic, &insData_)),
                pwm_(new Pwm(&htim1, TIM_CHANNEL_2))
    {
        pwm_->start();
        pwm_->setDutyCycle(1500);
    }
    virtual ~Shooter() {}

    virtual void stop() = 0;
    virtual void enter() = 0;
    virtual void updateSelf() = 0;


    void update(void *_param)
    {
        if (xQueueReceive((((MsgBus_s *)_param)->shooterQueue), &smsg, 0) ==
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

    bool isLaunched = false;
    static constexpr float loopsRange = -170.f;//向下最多167.5圈
    static constexpr float SCREW_LOOP_DEADBAND = 0.5f;
    static constexpr float loopAcc = 3.5f;//丝杆分部转动每次最大圈圈数
    static constexpr float angTurnwheel_Load[5] = {4.093f, 3.155f, 1.040f, 5.242f, 4.093f};
    static constexpr float angTurnwheel_reLoad[5] = {4.093f, (3.155f + PI/3.f), (1.040f + PI/3.f), (5.242f + PI/4.f), 4.093f};
//----------------------Constant state--------------------------------------
    bool dartLoaded = false;
    bool screwBack = false;
//--------------------------------------------------------------------------
    ShooterMsg_s smsg = {};
    
protected:
    StateFactory<SHOOTER::FSMState_e> stateFactory_;
    INS_SYS::INSData_s insData_ = {};
    Pwm *pwm_;
    Subscriber<INS_SYS::INSData_s> *insSub_;


private:
};
