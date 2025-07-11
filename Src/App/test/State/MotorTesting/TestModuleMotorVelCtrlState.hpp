#pragma once
#include "TestModule.hpp"

class MotorVelCtrlState : public FSMState {
public:
    MotorVelCtrlState(const TestModule *_testModule) : testModule_(_testModule)
    {
        setStateName("MotorVelCtrl");
    };

    void enter() override { LOG::info("MotorVelCtrl", " enter"); }

    void run() override
    {
        testModule_->testGM6020Motor->cmd(PINYMOTOR::MotorCmdType_e::ON);
        // testModule_->testGM3510Motor->cmd(PINYMOTOR::MotorCmdType_e::SET_VEL,
        //                                   1);
        // testModule_->testGM6020Motor->cmd(PINYMOTOR::MotorCmdType_e::SET_VEL,
        //                                   0.5f);
        testModule_->testGM6020Motor->cmd(PINYMOTOR::MotorCmdType_e::SET_TORQ,
                                          0.3f);
    };
    void exit() override { LOG::info("MotorVelCtrl", " exit"); }

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