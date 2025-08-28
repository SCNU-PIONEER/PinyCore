#pragma once

#include "FSMState.hpp"
#include "StmLog.hpp"
#include "TopicRouter.hpp"

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

    Chassis()
            : deltaYawSub_(new Subscriber<DeltaYawMsg_s>(
                      1, &TopicRouter::instance().deltaYawTopic, &deltaYawMsg_))
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
        if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) ==
            pdTRUE) {
        };

        loadDeltaYaw();

        actuator_->updateSelf();

        this->stateFactory_.update();
    }

    void loadDeltaYaw()
    {
        // Master Board method
        if (deltaYawSub_->receive()) {
            deltaYaw_ = deltaYawMsg_.deltaYaw;
        }
        // TODO: Slave Board method
    };

    ChassisMsg_s msg = {};

protected:
    StateFactory stateFactory_;

    DeltaYawMsg_s deltaYawMsg_;
    Subscriber<DeltaYawMsg_s> *deltaYawSub_;

    float deltaYaw_ = 0.f;

private:
    Actuator *actuator_ = static_cast<Actuator *>(this);
};
