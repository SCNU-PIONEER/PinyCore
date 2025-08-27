#pragma once

#include "FSMState.hpp"
#include "StmLog.hpp"
#include "MsgImpl.hpp"

#include "IMotor.hpp"

namespace CHASSIS {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN };
}
template <typename Actuator> class Chassis {
public:
    static constexpr float MAX_VX_SPEED = 2.f;
    static constexpr float MAX_VY_SPEED = 2.f;
    static constexpr float MAX_WZ_SPEED = 3.f;
    static constexpr float MOUSE_CAL = 200.f;

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
        if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) ==
            pdTRUE) {
        };
        actuator_->update();
        this->stateFactory_.update();
    }


    ChassisMsg_s msg = {};

    float yawAng() const { return yawAng_; }
    void setYawZero(float _zero) { yawZero_ = _zero; }

protected:
    StateFactory stateFactory_;

    float yawAng_ = 0.f;
    float yawZero_ = 0.f;

private:
    Actuator *actuator_ = static_cast<Actuator *>(this);
};
