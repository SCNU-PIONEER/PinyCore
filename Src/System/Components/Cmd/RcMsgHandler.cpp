#include "RcMsgHandler.hpp"
#include <algorithm>
#include "sdkconfig.h"
#include "Chassis.hpp"
#include "Smooth.hpp"

#include <cstring>

void rcMsgHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
    extern UART_HandleTypeDef RC_UART;
    rc_.init(&RC_UART, _event);
}


void rcMsgHandler::handle()
{
    rc_.parseData();
    RC::RcRawMsg_t rcData = rc_.getData();

    chassisMsg cmsg;
    gimbalMsg gmsg;
    armMsg amsg;

    rcMsg_.rx +=
            std::clamp(((float)rcData.rc.ch0 * T_ACC_CNT / 660.0f) - rcMsg_.rx,
                       -S_CURVE_ACC, S_CURVE_ACC);
    rcMsg_.ry +=
            std::clamp(((float)rcData.rc.ch1 * T_ACC_CNT / 660.0f) - rcMsg_.ry,
                       -S_CURVE_ACC, S_CURVE_ACC);
    rcMsg_.lx +=
            std::clamp(((float)rcData.rc.ch2 * T_ACC_CNT / 660.0f) - rcMsg_.lx,
                       -S_CURVE_ACC, S_CURVE_ACC);
    rcMsg_.ly +=
            std::clamp(((float)rcData.rc.ch3 * T_ACC_CNT / 660.0f) - rcMsg_.ly,
                       -S_CURVE_ACC, S_CURVE_ACC);

    rcMsg_.rSwitch = rcData.rc.switchRight;
    rcMsg_.lSwitch = rcData.rc.switchLeft;

    rcMsg_.xMove = rcData.mouse.x;
    rcMsg_.yMove = rcData.mouse.y;
    rcMsg_.zRoller = rcData.mouse.z;

    if (rcData.rc.switchRight == RC_SW_DOWN) {
        cmsg.state = State_e::STOP;
    } else if (rcData.rc.switchRight == RC_SW_MID) {
        if (rcData.rc.switchLeft == RC_SW_DOWN) {
            cmsg.state = State_e::NORMAL;
            cmsg.vx = s_curve(Chassis::MAX_VX_SPEED, rcMsg_.ry); // Scale to m/s
            cmsg.vy = s_curve(Chassis::MAX_VY_SPEED, rcMsg_.rx); // Scale to m/s
            cmsg.wz = s_curve(Chassis::MAX_WZ_SPEED, rcMsg_.lx); // Scale to m/s
        } else if (rcData.rc.switchLeft == RC_SW_MID) {
            amsg.state = State_e::NORMAL;
            amsg.j1 -= rcMsg_.rx / 314 / 300;
            amsg.j2 += rcMsg_.ry / 314 / 150;
            amsg.j3 -= rcMsg_.ly / 314 / 100;
            amsg.j4 += rcMsg_.lx / 314 / 100;
            amsg.j5 -= static_cast<float>(rcMsg_.zRoller) / 314 / 100;
            amsg.j6 += static_cast<float>(rcMsg_.xMove) / 314 / 100;
            amsg.j7 += static_cast<float>(rcMsg_.yMove) / 314 / 50;
        }
    } else {
        cmsg.state = State_e::STOP;
        amsg.state = State_e::STOP;
        cmsg.vx = 0.f;
        cmsg.vy = 0.f;
        cmsg.wz = 0.f;
    }

    memcpy(&rcMsgPrev_, &rcMsg_, sizeof(rcMsg_));

    notify(&cmsg, msgBus_->chassisQueue);
    notify(&amsg, msgBus_->armQueue);
}


void rcMsgHandler::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}
