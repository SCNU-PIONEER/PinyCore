#include "RcMsgHandler.hpp"
#include <algorithm>
#include "sdkconfig.h"

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

    rcMsg_.rx =
            std::clamp((float)rcData.rc.ch0 * T_ACC_CNT / 660.0f - rcMsg_.rx,
                       -S_CURVE_ACC, S_CURVE_ACC);
    rcMsg_.ry =
            std::clamp((float)rcData.rc.ch1 * T_ACC_CNT / 660.0f - rcMsg_.rx,
                       -S_CURVE_ACC, S_CURVE_ACC);
    rcMsg_.lx =
            std::clamp((float)rcData.rc.ch2 * T_ACC_CNT / 660.0f - rcMsg_.rx,
                       -S_CURVE_ACC, S_CURVE_ACC);
    rcMsg_.ly =
            std::clamp((float)rcData.rc.ch3 * T_ACC_CNT / 660.0f - rcMsg_.rx,
                       -S_CURVE_ACC, S_CURVE_ACC);

    rcMsg_.rSwitch = rcData.rc.switchRight;
    rcMsg_.lSwitch = rcData.rc.switchLeft;

    rcMsg_.xMove = rcData.mouse.x;
    rcMsg_.yMove = rcData.mouse.y;
    rcMsg_.zRoller = rcData.mouse.z;

    if (rcData.rc.switchRight == RC_SW_DOWN) {
        cmsg.state = State_e::stop;
    } else if (rcData.rc.switchRight == RC_SW_MID) {
        cmsg.state = State_e::run;
        cmsg.vx = rcMsg_.ry; // Scale to m/s
        cmsg.vy = rcMsg_.ry; // Scale to m/s
        cmsg.wz = rcMsg_.lx; // Scale to m/s

    } else {
        cmsg.state = State_e::stop; // Default state
    }

    memcpy(&rcMsgPrev_, &rcMsg_, sizeof(rcMsg_));

    notify(&cmsg, msgBus_->chassisQueue);
}


void rcMsgHandler::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}
