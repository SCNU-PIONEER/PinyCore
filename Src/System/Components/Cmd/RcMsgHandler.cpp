#include "RcMsgHandler.hpp"
#include <algorithm>
#include "sdkconfig.h"

void rcMsgHandler::init(EventGroupHandle_t _event)
{
    event = _event;
    extern UART_HandleTypeDef RC_UART;
    rc_.init(&RC_UART, _event);
}


void rcMsgHandler::handle()
{
    rc_.parseData();
    RC::rc_ctrl_t rcData = rc_.getData();


    rocker.rx =
            std::clamp((float)rcData.rc.ch0 * T_ACC_CNT / 660.0f - rocker.rx,
                       -S_CURVE_ACC, S_CURVE_ACC);
    rocker.ry =
            std::clamp((float)rcData.rc.ch1 * T_ACC_CNT / 660.0f - rocker.rx,
                       -S_CURVE_ACC, S_CURVE_ACC);
    rocker.lx =
            std::clamp((float)rcData.rc.ch2 * T_ACC_CNT / 660.0f - rocker.rx,
                       -S_CURVE_ACC, S_CURVE_ACC);
    rocker.ly =
            std::clamp((float)rcData.rc.ch3 * T_ACC_CNT / 660.0f - rocker.rx,
                       -S_CURVE_ACC, S_CURVE_ACC);

    if (rcData.rc.switch_right == RC_SW_DOWN) {
        cmsg.state = State_e::stop;
    } else if (rcData.rc.switch_right == RC_SW_MID) {
        cmsg.state = State_e::run;
        cmsg.vx = rocker.ry; // Scale to m/s
        cmsg.vy = rocker.ry; // Scale to m/s
        cmsg.wz = rocker.lx; // Scale to m/s

    } else {
        cmsg.state = State_e::stop; // Default state
    }

    notify(cmsg);
}

void rcMsgHandler::addObserver(IObserver *observer)
{
    {
        if (observer) {
            observers.push_back(observer);
        }
    }
}

void rcMsgHandler::notify(Msg &_msg)
{
    for (auto *observer : observers) {
        if (observer) {
            observer->getMsg(_msg);
        }
    }
}
