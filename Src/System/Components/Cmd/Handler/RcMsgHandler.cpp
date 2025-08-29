#include "RcMsgHandler.hpp"
#include <algorithm>
#include "sdkconfig.h"

#include "Mecanum.hpp"
#include "Standard.hpp"
#include CHASSIS_FILE
#include GIMBAL_FILE

#include "Smooth.hpp"

#include <cmath>
#include <cstring>

void RcMsgHandler::updateRocker(float &_target, float _channel)
{
    float delta = (_channel * T_ACC_CNT / 660.0f) - _target;
    _target += std::fmax(-S_CURVE_ACC, std::fmin(S_CURVE_ACC, delta));
}

void RcMsgHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
    extern UART_HandleTypeDef RC_UART;
    rc_.init(&RC_UART, _event);
}


void RcMsgHandler::handle()
{
    rc_.parseData();
    RC::RcRawMsg_t rcData = rc_.getData();

    updateRocker(rcMsg_.rx, (float)rcData.rc.ch0);
    updateRocker(rcMsg_.ry, (float)rcData.rc.ch1);
    updateRocker(rcMsg_.lx, (float)rcData.rc.ch2);
    updateRocker(rcMsg_.ly, (float)rcData.rc.ch3);

    rcMsg_.rSwitch = rcData.rc.switchRight;
    rcMsg_.lSwitch = rcData.rc.switchLeft;

    rcMsg_.xMove = rcData.mouse.x;
    rcMsg_.yMove = rcData.mouse.y;
    rcMsg_.zRoller = rcData.mouse.z;

    masterHandle();
}

void RcMsgHandler::chassisHandle()
{
    if (rcMsg_.rSwitch == RC_SW_DOWN) {
        cmsg_.state = CHASSIS::FSMState_e::STOP;
        cmsg_.vx = 0.f;
        cmsg_.vy = 0.f;
        cmsg_.yaw = 0.f;
    } else if (rcMsg_.rSwitch == RC_SW_MID) {
        cmsg_.state = CHASSIS::FSMState_e::RUN;
        cmsg_.vx = s_curve(ROCKER_VX_GAIN, rcMsg_.ry);   // Scale to m/s
        cmsg_.vy = -s_curve(ROCKER_VY_GAIN, rcMsg_.rx);  // Scale to m/s
        float dyaw = s_curve(ROCKER_WZ_GAIN, rcMsg_.lx); // Scale to m/s
        cmsg_.yaw += dyaw;
        constexpr float TWO_PI = 2.0f * M_PI;
        if (cmsg_.yaw > M_PI) {
            cmsg_.yaw -= TWO_PI;
        } else if (cmsg_.yaw < -M_PI) {
            cmsg_.yaw += TWO_PI;
        }
    } else {
        return;
    }
    notify(&cmsg_, msgBus_->chassisQueue);
}

void RcMsgHandler::masterHandle()
{
    if (rcMsg_.rSwitch == RC_SW_DOWN) {
        // gimbal
        gmsg_.state = GIMBAL::FSMState_e::STOP;
        gmsg_.pitch = gmsg_.yaw = 0.f;
        // chassis
        cmsg_.state = CHASSIS::FSMState_e::STOP;
        cmsg_.vx = cmsg_.vy = cmsg_.yaw = 0.f;
    } else if (rcMsg_.rSwitch == RC_SW_MID) {
        // gimbal
        gmsg_.state = GIMBAL::FSMState_e::RUN;
        float dpitch = -s_curve(ROCKER_PITCH_GAIN, rcMsg_.ly);
        float dyaw = -s_curve(ROCKER_YAW_GAIN, rcMsg_.lx);
        gmsg_.pitch += dpitch;
        gmsg_.yaw += dyaw;
        constexpr float TWO_PI = 2.0f * M_PI;
        if (gmsg_.pitch > M_PI) {
            gmsg_.pitch -= TWO_PI;
        } else if (gmsg_.pitch < -M_PI) {
            gmsg_.pitch += TWO_PI;
        }
        if (gmsg_.yaw > M_PI) {
            gmsg_.yaw -= TWO_PI;
        } else if (gmsg_.yaw < -M_PI) {
            gmsg_.yaw += TWO_PI;
        }
        // chassis
        cmsg_.state = CHASSIS::FSMState_e::RUN;
        cmsg_.vx = s_curve(ROCKER_VX_GAIN, rcMsg_.ry);
        cmsg_.vy = -s_curve(ROCKER_VY_GAIN, rcMsg_.rx);
        cmsg_.yaw = gmsg_.yaw;
    } else {
        return;
    }
    notify(&cmsg_, msgBus_->chassisQueue);
    notify(&gmsg_, msgBus_->gimbalQueue);
}

void RcMsgHandler::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}
