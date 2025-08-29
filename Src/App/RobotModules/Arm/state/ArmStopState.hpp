#pragma once

#include "Arm.hpp"

namespace ARM {

class StopState : public FSMState {
public:
    StopState(Arm &_arm) : arm_(_arm) {};

    void enter() override
    {
        arm_.motors.stop();
        LOG::info("Stop", "enter");
    }

    void run() override { arm_.motors.stop(); }

    void exit() override { LOG::info("Stop", "exit"); }

    uint8_t checkChange() override
    {
        // 根据指令来源和状态切换
        if (arm_.msg_.state == State_e::NORMAL) {
            if (arm_.msg_.source == ControlSource_e::RC) {
                return static_cast<uint8_t>(
                        FSMState_e::NORMAL); // 切换到正常状态
            } else if (arm_.msg_.source == ControlSource_e::TP) {
                return static_cast<uint8_t>(
                        FSMState_e::TEACH); // 切换到示教状态
            }
        } else if (arm_.msg_.state == State_e::PLAN) {
            return static_cast<uint8_t>(FSMState_e::PLAN);
        }
        // 否则保持停止状态
        return static_cast<uint8_t>(FSMState_e::STOP);
    }


private:
    Arm &arm_;
};

} // namespace ARM
