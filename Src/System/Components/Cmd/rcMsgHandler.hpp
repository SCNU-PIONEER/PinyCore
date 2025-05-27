#pragma once

#include "rc.hpp"
#include "handler.hpp"
#include "msgImpl.hpp"
#include <algorithm>

typedef struct {
    float rx;
    float ry;
    float lx;
    float ly;
} rocker_t;

#define S_CURVE_ACC 2.0f
#define T_ACC_CNT   100


class rcMsgHandler : public Handler {
public:
    rcMsgHandler(chassisMsg *cmsg, gimbalMsg *gmsg) : Handler(cmsg, gmsg) {}

    void init(UART_HandleTypeDef *uart) { rc_.init(uart); };

    inline uint8_t parseData() override
    {
        return rc_.parseData() != RC_VERIFY_ERR;
    }

    inline void handle() override
    {
        RC::rc_ctrl_t rcData = rc_.getData();

        rocker.rx = std::clamp((float)rcData.rc.ch0 * T_ACC_CNT / 660.0f -
                                       rocker.rx,
                               -S_CURVE_ACC, S_CURVE_ACC);
        rocker.ry = std::clamp((float)rcData.rc.ch1 * T_ACC_CNT / 660.0f -
                                       rocker.rx,
                               -S_CURVE_ACC, S_CURVE_ACC);
        rocker.lx = std::clamp((float)rcData.rc.ch2 * T_ACC_CNT / 660.0f -
                                       rocker.rx,
                               -S_CURVE_ACC, S_CURVE_ACC);
        rocker.ly = std::clamp((float)rcData.rc.ch3 * T_ACC_CNT / 660.0f -
                                       rocker.rx,
                               -S_CURVE_ACC, S_CURVE_ACC);

        if (rcData.rc.switch_right == RC_SW_DOWN) {
            cmsg->state = State_e::stop;
        } else if (rcData.rc.switch_right == RC_SW_MID) {
            cmsg->state = State_e::run;
            cmsg->vx = rocker.ry; // Scale to m/s
            cmsg->vy = rocker.ry; // Scale to m/s
            cmsg->wz = rocker.lx; // Scale to m/s

        } else {
            cmsg->state = State_e::stop; // Default state
        }
    }

private:
    RC::Rc &rc_ = RC::Rc::instance();
    rocker_t rocker;
};
