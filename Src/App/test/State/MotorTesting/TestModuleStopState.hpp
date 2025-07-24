#pragma once

#include "TestModule.hpp"

namespace TEST {

class MotorStopState : public FSMState {
public:
    MotorStopState(const TestModule *_testModule) : testModule_(_testModule)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::MOTOR_STOP));
    };

    void enter() override { LOG::info("MOTOR_STOP", " enter"); }

    void run() override
    {
        testModule_->testGM6020Motor->cmd(PINYMOTOR::MotorCmdType_e::OFF);
    };

    void exit() override { LOG::info("MOTOR_STOP", " exit"); }

    uint8_t checkChange() override
    {
        if (testModule_->rcMsg.rc.switchRight == RC_SW_DOWN) {
            return static_cast<uint8_t>(FSMState_e::MOTOR_STOP);
        } else if (testModule_->rcMsg.rc.switchRight == RC_SW_MID) {
            return static_cast<uint8_t>(FSMState_e::MOTOR_VEL_CTRL);
        } else if (testModule_->rcMsg.rc.switchRight == RC_SW_UP) {
            return static_cast<uint8_t>(FSMState_e::MOTOR_POS_CTRL);
        } else {
            return -1;
        }
    }

private:
    const TestModule *testModule_;
};
}
