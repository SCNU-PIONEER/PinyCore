#pragma once

#include "TestModule.hpp"

using namespace TEST;

class MotorPosCtrlState : public FSMState {
public:
    MotorPosCtrlState(const TestModule *_testModule) : testModule_(_testModule)
    {
        setStateName(static_cast<uint8_t>(fsmState_e::MotorPosCtrl));
    };

    void enter() override { LOG::info("MotorPosCtrl", " enter"); }

    void run() override
    {
        testModule_->testGM6020Motor->cmd(PINYMOTOR::MotorCmdType_e::ON);
        testModule_->testGM6020Motor->cmd(PINYMOTOR::MotorCmdType_e::SET_POS,
                                          1);
    };

    void exit() override { LOG::info("MotorPosCtrl", " exit"); }

    uint8_t checkChange() override
    {
        if (testModule_->rcMsg.rc.switchRight == RC_SW_DOWN) {
            return static_cast<uint8_t>(fsmState_e::MotorStop);
        } else if (testModule_->rcMsg.rc.switchRight == RC_SW_MID) {
            return static_cast<uint8_t>(fsmState_e::MotorVelCtrl);
        } else if (testModule_->rcMsg.rc.switchRight == RC_SW_UP) {
            return static_cast<uint8_t>(fsmState_e::MotorPosCtrl);
        }
        return -1;
    }

private:
    const TestModule *testModule_;
};
