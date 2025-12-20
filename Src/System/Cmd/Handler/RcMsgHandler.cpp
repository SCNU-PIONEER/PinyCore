#include "RcMsgHandler.hpp"
#include "sdkconfig.h"

#include "remoteControl.hpp"

#include "Chassis.hpp"
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

void RcMsgHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
}


void RcMsgHandler::handle()
{
    rc->parse();
#if EXTENSION_DT7 == 1
    RcRawMsg_t *rcData = static_cast<RcRawMsg_t *>(rc->getData());

    updateRocker(rcMsg_.rx, (float)rcData->rc.ch0);
    updateRocker(rcMsg_.ry, (float)rcData->rc.ch1);
    updateRocker(rcMsg_.lx, (float)rcData->rc.ch2);
    updateRocker(rcMsg_.ly, (float)rcData->rc.ch3);

    rcMsg_.rSwitch = rcData->rc.switchRight;
    rcMsg_.lSwitch = rcData->rc.switchLeft;

    rcMsg_.xMove = rcData->mouse.x;
    rcMsg_.yMove = rcData->mouse.y;
    rcMsg_.zRoller = rcData->mouse.z;
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
    masterHandle();
}

void RcMsgHandler::chassisHandle() {}

void RcMsgHandler::masterHandle()
{
    ChassisMsg_s chassisMsg{};
    GimbalMsg_s gimbalMsg{};
    if (rcMsg_.rSwitch == RC_SW_DOWN) {
        // gimbal
        gimbalMsg.state = GIMBAL::FSMState_e::STOP;
        gimbalMsg.pitch = gimbalMsg.yaw = 0.f;
        // chassis
        chassisMsg.state = CHASSIS::FSMState_e::STOP;
        chassisMsg.vx = chassisMsg.vy = 0.f;
    } else if (rcMsg_.rSwitch == RC_SW_MID) {
        // gimbal
        gimbalMsg.state = GIMBAL::FSMState_e::RUN;
        gimbalMsg.pitch = -SMOOTH::sCurve(ROCKER_PITCH_GAIN, rcMsg_.ly, T_ACC_CNT);
        gimbalMsg.yaw = -SMOOTH::sCurve(ROCKER_YAW_GAIN, rcMsg_.lx, T_ACC_CNT);
        // chassis
        chassisMsg.state = CHASSIS::FSMState_e::RUN;
        chassisMsg.vx = SMOOTH::sCurve(ROCKER_VX_GAIN, rcMsg_.ry, T_ACC_CNT);
        chassisMsg.vy = -SMOOTH::sCurve(ROCKER_VY_GAIN, rcMsg_.rx, T_ACC_CNT);

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
    notify(&chassisMsg, msgBus_->chassisQueue);
    notify(&gimbalMsg, msgBus_->gimbalQueue);
}

void RcMsgHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }
