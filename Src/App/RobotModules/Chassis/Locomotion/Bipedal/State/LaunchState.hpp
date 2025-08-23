#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS::BIPEDAL {
class LaunchState : public FSMState {
    static constexpr float LAUNCH_DPHI = 0.5f * 0.001f; // per ms
    static constexpr float LAUNCH_PHI = -0.3f;
    static constexpr float PHI_THRESHOLD = 0.1f;
    static constexpr float LAUNCH_L = 0.3f;
    static constexpr float L_THRESHOLD = 0.05f;

public:
    LaunchState(Bipedal *_chassis) : chassis_(_chassis)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::LAUNCH));
    }

    void enter() final
    {
        isLaunchFinish_ = false;
        chassis_->enter();
        LOG::info("ChassisLaunch", "enter");
    }

    void run() final
    {
        chassis_->setL(LAUNCH_L);
        if (chassis_->isOverturn()) {
            // giant circle
            chassis_->ctrlHipRot(LAUNCH_DPHI);
        } else {
            chassis_->ctrlHip(LAUNCH_PHI);
            // no wait time
            isLaunchFinish_ = chassis_->isLaunched(LAUNCH_PHI, PHI_THRESHOLD,
                                                   LAUNCH_L, L_THRESHOLD);
        }
    }

    void exit() final { LOG::info("ChassisLaunch", " exit"); }


    uint8_t checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (isLaunchFinish_)
            return static_cast<uint8_t>(FSMState_e::RUN);
        // TODO: the other state
        else
            return 0;
    }

private:
    Bipedal *chassis_;
    bool isLaunchFinish_ = false;
};

} // namespace CHASSIS::BIPEDAL
