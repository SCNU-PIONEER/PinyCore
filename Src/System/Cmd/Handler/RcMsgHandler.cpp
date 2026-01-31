#include "RcMsgHandler.hpp"
#include "DT7Msg.hpp"
#include "sdkconfig.h"

#include "remoteControl.hpp"

#include "Chassis.hpp"
#include "Arm.hpp"
#ifdef GIMBAL_TYPE
#include GIMBAL_FILE
#endif

#include "Smooth.hpp"

#include <cmath>
#include <cstring>

RcMsgHandler::RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event)
{
#if EXTENSION_DT7 == 1
    rc = std::make_unique<Rc>(_huart, _event);
#elif EXTENSION_ET08A == 1
    rc = std::make_unique<ET08A>(_huart, _event);
#endif
};


void RcMsgHandler::updateRocker(float &_target, float _channel)
{
    float delta = (_channel * T_ACC_CNT / 660.0f) - _target;
    _target += std::fmax(-S_CURVE_ACC, std::fmin(S_CURVE_ACC, delta));
}

void RcMsgHandler::kupdateRocker(float &_target, float _channel)
{
    float delta = (_channel * T_ACC_CNT) - _target;
    _target += std::fmax(-S_CURVE_ACC, std::fmin(S_CURVE_ACC, delta));
}

uint32_t RcMsgHandler::isKeyPressed(uint16_t _key) { return ((rcData->keyboard.keyCode & _key) == _key); }
//TODO:更新过去遥控输入状态状态
uint32_t RcMsgHandler::isKeyLastPressed(uint16_t _key) { return ((rcData->keyboard.lastKeyCode & _key) == _key); }

void RcMsgHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
}

void RcMsgHandler::handle()
{
    rc->parse();
#if EXTENSION_DT7 == 1
    rcData = static_cast<RcRawMsg_t *>(rc->getData());

    updateRocker(rcMsg_.rx, (float)rcData->rc.ch0);
    updateRocker(rcMsg_.ry, (float)rcData->rc.ch1);
    updateRocker(rcMsg_.lx, (float)rcData->rc.ch2);
    updateRocker(rcMsg_.ly, (float)rcData->rc.ch3);

    rcMsg_.rSwitch = rcData->rc.switchRight;
    rcMsg_.lSwitch = rcData->rc.switchLeft;

    rcMsg_.xMove = rcData->mouse.x;
    rcMsg_.yMove = rcData->mouse.y;
    rcMsg_.zRoller = rcData->mouse.z;
    rcMsg_.rPress = rcData->mouse.pressRight;
    rcMsg_.lPress = rcData->mouse.pressLeft;

#elif EXTENSION_ET08A == 1
    ET08A::RcData_s *rcData = static_cast<ET08A::RcData_s *>(rc->getData());

    updateRocker(rcMsg_.rx, (float)rcData->ch0);
    updateRocker(rcMsg_.ry, (float)rcData->ch1);
    updateRocker(rcMsg_.lx, (float)rcData->ch2);
    updateRocker(rcMsg_.ly, (float)rcData->ch3);

    rcMsg_.rSwitch = static_cast<uint8_t>(rcData->switchRight);
    rcMsg_.lSwitch = static_cast<uint8_t>(rcData->switchLeft);

    rcMsg_.xMove = 0;
    rcMsg_.yMove = 0;
    rcMsg_.zRoller = 0;
#endif

#if APP_USE_GIMBAL == 1
    masterHandle();
#else
    chassisHandle();
#endif
}

//TODO：键盘
#if APP_USE_GIMBAL == 1
void RcMsgHandler::masterHandle()
{
    ChassisMsg_s chassisMsg{};
    GimbalMsg_s gimbalMsg{};
    ArmMsg_s armMsg{};

#if APP_USE_KEY == 1
    /*
gimbal:STOP,RUN,
chassis:STOP,RUN,SPIN
arm:ONECLICK,MODE,STOP,NORMAL,PLAN,TEACH,夹爪
*/
    //机械臂控制使用自定义控制器
    /*————————————————————————————————————————————————chassis——————————————————————————————————————————————*/
    float accK = 1.f, lxCommand = 0;
    chassisMsg.state = CHASSIS::FSMState_e::RUN;
    //加速
    if (isKeyPressed(KEY_SHIFT))
        accK = 1.5f;
    else
        accK = 1.f;

    //前后左右
    if (isKeyPressed(KEY_W)) {
        kupdateRocker(rcMsg_.ry, 1); //加速向前，ry 增加并趋近于最大值。
    } else if (isKeyPressed(KEY_S)) {
        kupdateRocker(rcMsg_.ry, -1); //加速向后，ry 减少并趋近于最小值。
    } else {
        kupdateRocker(rcMsg_.ry, 0); //自然减速回中，ry 向 0 值靠近。
    }

    if (isKeyPressed(KEY_A)) {
        kupdateRocker(rcMsg_.rx, -1);
    } else if (isKeyPressed(KEY_D)) {
        kupdateRocker(rcMsg_.rx, 1);
    } else {
        kupdateRocker(rcMsg_.rx, 0);
    }

    //旋转
    if (rcMsg_.rPress == 1)
        lxCommand = 1;
    else if (rcMsg_.lPress == 1)
        lxCommand = -1;
    else
        lxCommand = 0;

    kupdateRocker(rcMsg_.lx, lxCommand);

    chassisMsg.vx = SMOOTH::sCurve(accK * Chassis::MAX_VX_SPEED, rcMsg_.ry, T_ACC_CNT);  // Scale to m/s
    chassisMsg.vy = SMOOTH::sCurve(accK * Chassis::MAX_VY_SPEED, -rcMsg_.rx, T_ACC_CNT); // Scale to m/s
    chassisMsg.yaw = SMOOTH::sCurve(accK * Chassis::MAX_WZ_SPEED, rcMsg_.lx, T_ACC_CNT); // Scale to m/s
    /*——————————————————————————————————————————————————————————————————————————————————————————————————————————*/
    //单双臂
    if (isKeyPressed(KEY_SHIFT) && !isKeyLastPressed(KEY_SHIFT)) {
        if (isKeyLastPressed(KEY_Z) && !isKeyPressed(KEY_Z)) {
            armMsg.mode = ARM::ArmMode_e::LEFT;
        }
        if (isKeyLastPressed(KEY_X) && !isKeyPressed(KEY_X)) {
            armMsg.mode = ARM::ArmMode_e::RIGHT;
        }
        if (isKeyLastPressed(KEY_C) && !isKeyPressed(KEY_C)) {
            armMsg.mode = ARM::ArmMode_e::TWO;
        }
    }
    //一键: 取矿*3，从矿仓取矿，公路区跨越, 夹爪
    if (isKeyPressed(KEY_Q))
        armMsg.oneClick = ARM::OneClick_e::ORE_PICK1;
    if (isKeyPressed(KEY_E))
        armMsg.oneClick = ARM::OneClick_e::ORE_PICK2;
    if (isKeyPressed(KEY_R))
        armMsg.oneClick = ARM::OneClick_e::ORE_PICK3;
    if (isKeyPressed(KEY_F))
        armMsg.oneClick = ARM::OneClick_e::HIGHWAY_CROSS;
    if (isKeyPressed(KEY_G))
        armMsg.oneClick = ARM::OneClick_e::ORE_BIN_EXTRACT;
    if (isKeyPressed(KEY_V) && !isKeyLastPressed(KEY_V))
        armMsg.switch1 = !armMsg.switch1;
#else

    if (rcMsg_.rSwitch == RC_SW_DOWN) {
        //arm
        armMsg.switch1 = false;
        armMsg.state = ARM::FSMState_e::STOP;
        // gimbal
        gimbalMsg.state = GIMBAL::FSMState_e::STOP;
        gimbalMsg.pitch = gimbalMsg.yaw = 0.f;
        // chassis
        chassisMsg.state = CHASSIS::FSMState_e::STOP;
        chassisMsg.vx = chassisMsg.vy = 0.f;
    } else if (rcMsg_.rSwitch == RC_SW_MID) {
        if (rcMsg_.lSwitch == RC_SW_DOWN) {
            // gimbal
            gimbalMsg.state = GIMBAL::FSMState_e::RUN;
            gimbalMsg.pitch = -SMOOTH::sCurve(ROCKER_PITCH_GAIN, rcMsg_.ly, T_ACC_CNT);
            gimbalMsg.yaw = -SMOOTH::sCurve(ROCKER_YAW_GAIN, rcMsg_.lx, T_ACC_CNT);
            // chassis
            chassisMsg.state = CHASSIS::FSMState_e::RUN;
            chassisMsg.vx = SMOOTH::sCurve(ROCKER_VX_GAIN, rcMsg_.ry, T_ACC_CNT);
            chassisMsg.vy = -SMOOTH::sCurve(ROCKER_VY_GAIN, rcMsg_.rx, T_ACC_CNT);

        } else if (rcMsg_.lSwitch == RC_SW_MID) {
            //arm
            armMsg.target[0] = -rcMsg_.lx / 314 / 200;
            armMsg.target[1] = rcMsg_.ly / 314 / 100;
            //夹爪
            if (rcData->rc.ch1 == 660)
                armMsg.switch1 = !armMsg.switch1;
        } else if (rcMsg_.lSwitch == RC_SW_UP) {
            //arm
            armMsg.target[2] = rcMsg_.ly / 314 / 150;  //pitch
            armMsg.target[3] = rcMsg_.lx / 314 / 50;   //yaw
            armMsg.target[4] = -rcMsg_.ry / 314 / 100; //pitch
            armMsg.target[5] = rcMsg_.rx / 314 / 50;   //roll

            armMsg.target[6] = rcMsg_.ly / 314 / 150;  //pitch
            armMsg.target[7] = -rcMsg_.lx / 314 / 50;  //yaw
            armMsg.target[8] = -rcMsg_.ry / 314 / 100; //pitch
            armMsg.target[9] = -rcMsg_.rx / 314 / 50;  //roll
        }
    } else if (rcMsg_.rSwitch == RC_SW_UP) {
        // gimbal
        gimbalMsg.state = GIMBAL::FSMState_e::RUN;
        gimbalMsg.pitch = -SMOOTH::sCurve(ROCKER_PITCH_GAIN, rcMsg_.ly, T_ACC_CNT);
        gimbalMsg.yaw = -SMOOTH::sCurve(ROCKER_YAW_GAIN, rcMsg_.lx, T_ACC_CNT);
        // chassis
        if (rcMsg_.lSwitch == RC_SW_DOWN) {
        } else {
            chassisMsg.state = CHASSIS::FSMState_e::SPIN;
        }
        chassisMsg.vx = SMOOTH::sCurve(ROCKER_VX_GAIN, rcMsg_.ry, T_ACC_CNT);
        chassisMsg.vy = -SMOOTH::sCurve(ROCKER_VY_GAIN, rcMsg_.rx, T_ACC_CNT);
    } else {
        return;
    }

#endif
    notify(&chassisMsg, msgBus_->chassisQueue);
    notify(&gimbalMsg, msgBus_->gimbalQueue);
    notify(&armMsg, msgBus_->armQueue);
}
#else
void RcMsgHandler::chassisHandle()
{
    ChassisMsg_s chassisMsg{};
    if (rcMsg_.rSwitch == RC_SW_DOWN) {
        chassisMsg.state = CHASSIS::FSMState_e::STOP;
        chassisMsg.vx = 0.f;
        chassisMsg.vy = 0.f;
        chassisMsg.yaw = 0.f;
    } else {
        if (rcMsg_.lSwitch == RC_SW_DOWN) {
            chassisMsg.state = CHASSIS::FSMState_e::RUN;
        } else {
            chassisMsg.state = CHASSIS::FSMState_e::SPIN;
        }
        chassisMsg.vx = SMOOTH::sCurve(ROCKER_VX_GAIN, rcMsg_.ry, T_ACC_CNT);
        chassisMsg.vy = -SMOOTH::sCurve(ROCKER_VY_GAIN, rcMsg_.rx, T_ACC_CNT);
        chassisMsg.yaw = SMOOTH::sCurve(ROCKER_YAW_GAIN, rcMsg_.lx, T_ACC_CNT);
    }
    memcpy(&rcMsgPrev_, &rcMsg_, sizeof(rcMsg_));
    notify(&chassisMsg, msgBus_->chassisQueue);
}

#endif

void RcMsgHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }
