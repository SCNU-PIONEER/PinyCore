#pragma once

#include "TestModule.hpp"

class MotorStopState : public FSMState {
public:
    MotorStopState(const TestModule *_testModule) : testModule_(_testModule)
    {
        setStateName("MotorStop");
    };

    void enter() override { LOG::info("MotorStop", " enter"); }

    void run() override
    {
        testModule_->testGM6020Motor->cmd(PINYMOTOR::MotorCmdType_e::OFF);
    };

    void exit() override { LOG::info("MotorStop", " exit"); }

    std::string checkChange() override
    {
        if (testModule_->rcMsg.rc.switchRight == RC_SW_DOWN) {
            return "MotorStop";
        } else if (testModule_->rcMsg.rc.switchRight == RC_SW_MID) {
            return "MotorVelCtrl";
        } else if (testModule_->rcMsg.rc.switchRight == RC_SW_UP) {
            return "MotorPosCtrl";
        } else {
            return "MotorStop";
        }
    }

private:
    const TestModule *testModule_;
};
