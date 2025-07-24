#pragma once
#include "TestModule.hpp"

namespace TEST {

class MotorVelCtrlState : public FSMState {
public:
    MotorVelCtrlState(const TestModule *_testModule) : testModule_(_testModule)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::MOTOR_VEL_CTRL));
    };

    void enter() override { LOG::info("MOTOR_VEL_CTRL", " enter"); }

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
    void exit() override { LOG::info("MOTOR_VEL_CTRL", " exit"); }

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
} // namespace TEST
