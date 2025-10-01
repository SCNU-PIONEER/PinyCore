#include "RcMsgHandler.hpp"
#include <algorithm>
#include "sdkconfig.h"

#ifdef CHASSIS_TYPE
#include CHASSIS_FILE
#endif
#ifdef GIMBAL_TYPE
#include GIMBAL_FILE
#endif

#include "Smooth.hpp"

#include <cmath>
#include <cstring>

RcMsgHandler::RcMsgHandler(UART_HandleTypeDef *_huart) : rc_(_huart) {};


void RcMsgHandler::updateRocker(float &_target, float _channel)
{
    float delta = (_channel * T_ACC_CNT / 660.0f) - _target;
    _target += std::fmax(-S_CURVE_ACC, std::fmin(S_CURVE_ACC, delta));
}

void RcMsgHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
    rc_.init(_event);
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

void RcMsgHandler::chassisHandle() {}

void RcMsgHandler::masterHandle() {}

void RcMsgHandler::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}
