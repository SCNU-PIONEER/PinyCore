#pragma once

#include "FSMState.hpp"
#include "TopicRouter.hpp"

#include "IMotor.hpp"
#include "MotorCommonMacros.hpp"
#include "INS.hpp"

namespace GIMBAL {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN };
}
template <typename Actuator> class Gimbal {
public:
    // some constant
    Gimbal()
            : insSub_(new Subscriber<INS_SYS::INSData_s>(
                      4, &TopicRouter::instance().insTopic, &insData_))
            , deltaYawPub_(new Publisher<DeltaYawMsg_s>(
                      &TopicRouter::instance().deltaYawTopic, &deltaYawMsg_))
    {
    }

    void stop()
    {
        for (auto &i : actuator_->motors_._) {
            i->cmd(PINYMOTOR::MotorCmdType_e::OFF);
        }
        actuator_->stopSelf();
    }

    void enter()
    {
        for (auto &i : actuator_->motors_._) {
            i->cmd(PINYMOTOR::MotorCmdType_e::ON);
        }
        actuator_->enterSelf();
    }

    void update(void *_param)
    {
        if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) ==
            pdTRUE) {
        };
        this->insSub_->receive();

        actuator_->updateEndYaw();
        actuator_->updateBaseYaw();

        deltaYawMsg_.deltaYaw = PINYMOTOR::getMinorArc(endYawAng_, baseYawAng_);
        deltaYawPub_->publish();

        actuator_->updateSelf();

        this->stateFactory_.update();
    }

    GimbalMsg_s msg = {};

    const INS_SYS::INSData_s &ins() const { return insData_; }

    float endYawAng() const { return endYawAng_; }

    float baseYawAng() const { return baseYawAng_; }

protected:
    StateFactory stateFactory_;

    INS_SYS::INSData_s insData_ = {};
    Subscriber<INS_SYS::INSData_s> *insSub_;

    // Master send deltaYaw to chassis
    DeltaYawMsg_s deltaYawMsg_{};
    Publisher<DeltaYawMsg_s> *deltaYawPub_;

    float endYawAng_ = 0.f;        // end's ang in earth
    float constDeltaYawArc_ = 0.f; // end to base's arc
    bool isConfirmYawZero_ = false;

    float baseYawAng_ = 0.f; // J0's ang in earth

private:
    Actuator *actuator_ = static_cast<Actuator *>(this);
};
