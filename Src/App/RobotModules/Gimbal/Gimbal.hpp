#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"

#include "IMotor.hpp"
#include "INS.hpp"

namespace GIMBAL {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN };
}
template <typename Actuator> class Gimbal {
public:
    // some constant
    void stop()
    {
        for (auto &i : actuator_->motors_._) {
            i->cmd(PINYMOTOR::MotorCmdType_e::OFF);
        }
        actuator_->stop();
    }

    void enter()
    {
        for (auto &i : actuator_->motors_._) {
            i->cmd(PINYMOTOR::MotorCmdType_e::ON);
        }
        actuator_->enter();
    }

    void update(void *_param)
    {
        if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) ==
            pdTRUE) {
        };
        this->insSub_->receive();
        updateEndYaw();
        updateBaseYaw();
        actuator_->update();
        this->stateFactory_.update();
    }

    void updateEndYaw() { actuator_->updateEndYaw(); }

    void updateBaseYaw() { actuator_->updateBaseYaw(); }

    GimbalMsg_s msg = {};

    const INS_SYS::INSData_s &ins() const { return insData_; }

    float endYawAng() const { return endYawAng_; }

    float baseYawAng() const { return baseYawAng_; }

protected:
    StateFactory stateFactory_;

    INS_SYS::INSData_s insData_ = {};
    Subscriber<INS_SYS::INSData_s> *insSub_;

    float endYawAng_ = 0.f;  // end's ang in earth
    float yawDeltaArc = 0.f; // end to base's arc
    bool isConfirmYawZero = false;

    float baseYawAng_ = 0.f; // J0's ang in earth

private:
    Actuator *actuator_ = static_cast<Actuator *>(this);
};
