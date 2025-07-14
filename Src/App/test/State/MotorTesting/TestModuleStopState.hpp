#pragma once

#include "TestModule.hpp"

using namespace TEST;

class MotorStopState : public FSMState {
public:
    MotorStopState(const TestModule *_testModule) : testModule_(_testModule)
    {
        setStateName(static_cast<uint8_t>(fsmState_e::MotorStop));
    };

    void enter() override { LOG::info("MotorStop", " enter"); }

    void run() override
    {
        testModule_->testGM6020Motor->cmd(PINYMOTOR::MotorCmdType_e::OFF);
    };

    void exit() override { LOG::info("MotorStop", " exit"); }

    uint8_t checkChange() override
    {
        if (testModule_->rcMsg.rc.switchRight == RC_SW_DOWN) {
            return static_cast<uint8_t>(fsmState_e::MotorStop);
        } else if (testModule_->rcMsg.rc.switchRight == RC_SW_MID) {
            return static_cast<uint8_t>(fsmState_e::MotorVelCtrl);
        } else if (testModule_->rcMsg.rc.switchRight == RC_SW_UP) {
            return static_cast<uint8_t>(fsmState_e::MotorPosCtrl);
        } else {
            return -1;
        }
    }

private:
    const TestModule *testModule_;
};
