#pragma once

#include "TestModule.hpp"

class MotorPosCtrlState : public FSMState {
public:
    MotorPosCtrlState(const TestModule *_testModule) : testModule_(_testModule)
    {
        setStateName("motorPosCtrl");
    };

    void enter() override { LOG::info("MotorPosCtrl", " enter"); }

    void run() override
    {
        testModule_->testGM3510Motor->cmd(PINYMOTOR::MotorCmdType_e::ON);
        testModule_->testGM3510Motor->cmd(PINYMOTOR::MotorCmdType_e::SET_POS,
                                          1);
    };

    void exit() override { LOG::info("MotorPosCtrl", " exit"); }

    std::string checkChange() override
    {
        if (testModule_->rcMsg.rc.switchRight == RC_SW_DOWN) {
            return "MotorStop";
        } else if (testModule_->rcMsg.rc.switchLeft == RC_SW_MID) {
            return "MotorVelCtrl";
        } else if (testModule_->rcMsg.rc.switchLeft == RC_SW_UP) {
            return "MotorPosCtrl";
        }
        return "";
    }

private:
    const TestModule *testModule_;
};
