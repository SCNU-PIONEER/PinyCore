#pragma once
#include "Mecanum.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"
#include "PidBasic.hpp"
#include "MotorCommonMacros.hpp"

namespace CHASSIS {
class RunState : public FSMState<FSMState_e> {
public:
    RunState(Mecanum &_chassis) : FSMState<FSMState_e>(FSMState_e::RUN), chassis_(_chassis) {};

    void enter() final
    {
        chassis_.enter();
        LOG::info("ChassisRun", " enter");
    }

    void run() final
    {
        Speed_u ref;

        ref.vx = chassis_.msg.vx;
        ref.vy = chassis_.msg.vy;
#if APP_USE_COMM == 1
        const float deltaYaw = PINYMOTOR::getMinorArc(chassis_.msg.yaw, 0.f);
        float targetYaw = (deltaYaw > M_PI / 2.f) ? M_PI : 0.f;
        ref.wz = wzAngPid_.calc(0, PINYMOTOR::getMinorArc(deltaYaw, targetYaw));
#else
        ref.wz = chassis_.msg.yaw;
#endif
        chassis_.ctrl(ref);
    }

    void exit() final
    {
        wzAngPid_.reset();
        LOG::info("ChassisRun", " exit");
    }


    FSMState_e checkChange() final
    {
        using enum FSMState_e;
        switch (chassis_.msg.state) {
        case RUN:
            return RUN;
        case SPIN:
            return SPIN;
        default:
            return STOP;
        }
    }

private:
    Mecanum &chassis_;
    PositionalPid wzAngPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };
};
} // namespace CHASSIS
