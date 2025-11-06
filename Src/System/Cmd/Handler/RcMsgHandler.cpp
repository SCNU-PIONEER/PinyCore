#include "RcMsgHandler.hpp"
#include "sdkconfig.h"

#include "remoteControl.hpp"

#ifdef CHASSIS_TYPE
#include CHASSIS_FILE
#endif
#ifdef GIMBAL_TYPE
#include GIMBAL_FILE
#endif
#ifdef ARMORLOADER_TYPE
#include ARMORLOADER_FILE
#endif
#ifdef ARMORBOOSTER_TYPE
#include ARMORBOOSTER_FILE
#endif

#include "Smooth.hpp"

#include <cmath>
#include <cstring>


RcMsgHandler::RcMsgHandler(UART_HandleTypeDef *_huart,
                           EventGroupHandle_t &_event)
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

    rcMsg_.wheelRocker = ((float)rcData->wheel - 1024.f) / 660.f;
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
    if (rcMsg_.rSwitch == RC_SW_DOWN) {
        // gimbal
        gimbalMsg_.state = GIMBAL::FSMState_e::STOP;
        gimbalMsg_.wPitch = gimbalMsg_.wYaw = 0.f;
        // chassis
        chassisMsg_.state = CHASSIS::FSMState_e::STOP;
        chassisMsg_.vx = chassisMsg_.vy = 0.f;
        // ArmorLoader
        armorLoaderMsg_.state = ARMORLOADER::FSMState_e::STOP;
        armorLoaderMsg_.fire = false;
        // ArmorBooster
        armorBoosterMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
    } else if (rcMsg_.rSwitch == RC_SW_MID) {
        // gimbal
        gimbalMsg_.state = GIMBAL::FSMState_e::RUN;
        gimbalMsg_.wPitch =
                -SMOOTH::sCurve(ROCKER_PITCH_GAIN, rcMsg_.ly, S_CURVE_ACC);
        gimbalMsg_.wYaw =
                -SMOOTH::sCurve(ROCKER_YAW_GAIN, rcMsg_.lx, S_CURVE_ACC);
        // chassis
        // chassisMsg_.state = CHASSIS::FSMState_e::IDLE;
        chassisMsg_.state = CHASSIS::FSMState_e::RUN;
        chassisMsg_.vx = SMOOTH::sCurve(ROCKER_VX_GAIN, rcMsg_.ry, S_CURVE_ACC);
        chassisMsg_.vy =
                -SMOOTH::sCurve(ROCKER_VY_GAIN, rcMsg_.rx, S_CURVE_ACC);
        if (rcMsg_.lSwitch == RC_SW_DOWN) {
            // ArmorLoader
            armorLoaderMsg_.state = ARMORLOADER::FSMState_e::STOP;
            // ArmorBooster
            armorBoosterMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
        } else if (rcMsg_.lSwitch == RC_SW_MID) {
            // ArmorLoader
            armorLoaderMsg_.state = ARMORLOADER::FSMState_e::AUTO;
            // ArmorBooster
            armorBoosterMsg_.state = ARMORBOOSTER::FSMState_e::ON;
        } else if (rcMsg_.lSwitch == RC_SW_UP) {
            // ArmorLoader
            armorLoaderMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
            // ArmorBooster
            armorBoosterMsg_.state = ARMORBOOSTER::FSMState_e::ON;
        }
        armorLoaderMsg_.fire = rcMsg_.wheelRocker > WHEEL_THESHORLD ||
                               rcMsg_.wheelRocker < -WHEEL_THESHORLD;

    } else if (rcMsg_.rSwitch == RC_SW_UP) {
        // gimbal
        gimbalMsg_.state = GIMBAL::FSMState_e::RUN;
        gimbalMsg_.wPitch =
                -SMOOTH::sCurve(ROCKER_PITCH_GAIN, rcMsg_.ly, S_CURVE_ACC);
        gimbalMsg_.wYaw =
                -SMOOTH::sCurve(ROCKER_YAW_GAIN, rcMsg_.lx, S_CURVE_ACC);
        // chassis
        if (rcMsg_.lSwitch == RC_SW_DOWN) {
            // chassisMsg_.state = CHASSIS::FSMState_e::TANK;
        } else {
            // chassisMsg_.state = CHASSIS::FSMState_e::SPIN;
        }
        chassisMsg_.vx = SMOOTH::sCurve(ROCKER_VX_GAIN, rcMsg_.ry, S_CURVE_ACC);
        chassisMsg_.vy =
                -SMOOTH::sCurve(ROCKER_VY_GAIN, rcMsg_.rx, S_CURVE_ACC);
        // ArmorLoader
        armorLoaderMsg_.state = ARMORLOADER::FSMState_e::STOP;
        armorLoaderMsg_.fire = false;
        // ArmorBooster
        armorBoosterMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
    } else {
        return;
    }
    notify(&chassisMsg_, msgBus_->chassisQueue);
    notify(&gimbalMsg_, msgBus_->gimbalQueue);
    notify(&armorLoaderMsg_, msgBus_->armorLoaderQueue);
    notify(&armorBoosterMsg_, msgBus_->armorBoosterQueue);
}

void RcMsgHandler::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}
