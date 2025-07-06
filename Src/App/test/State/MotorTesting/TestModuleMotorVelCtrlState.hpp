#pragma once
#include "TestModule.hpp"

class MotorVelCtrlState : public FSMState {
public:
    MotorVelCtrlState(const TestModule *_testModule) : testModule_(_testModule)
    {
        setStateName("motorVelCtrl");
    };

    void enter() override { LOG::info("MotorVelCtrl", " enter"); }

    void run() override
    {
        testModule_->testGM3510Motor->cmd(PINYMOTOR::MotorCmdType_e::ON);
        // testModule_->testGM3510Motor->cmd(PINYMOTOR::MotorCmdType_e::SET_VEL,
        //                                   1);
        testModule_->testGM3510Motor->cmd(PINYMOTOR::MotorCmdType_e::SET_ELEC,
                                          1);
    };
    void exit() override { LOG::info("MotorVelCtrl", " exit"); }

    std::string checkChange() override
    {
        if (testModule_->rcMsg.rc.switchRight == RC_SW_DOWN) {
            return "MotorStopCtrl";
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