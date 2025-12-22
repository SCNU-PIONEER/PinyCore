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

#if EXTENSION_DT7 == 1
#include "DT7.hpp"
#elif EXTENSION_ET08A == 1
#include "ET08A.hpp"
#elif EXTENSION_VT13 == 1
#include "VT13.hpp"
#endif

#include <cmath>
#include <cstring>
#include <bitset>


RcMsgHandler::RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event)
{
#if EXTENSION_DT7 == 1
    rc = std::make_unique<DT7>(_huart, _event, this->bit_);
#elif EXTENSION_ET08A == 1
    rc = std::make_unique<ET08A>(_huart, _event, this->bit_);
#elif EXTENSION_VT13 == 1
    rc = std::make_unique<VT13>(_huart, _event, this->bit_);
#endif
};

void RcMsgHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
}

void RcMsgHandler::handle()
{
    rc->parse();

    masterHandle();
}

/**
 * @brief Master handler for processing remote controller messages
 * 
 * @details Since remote controllers vary widely in channel configurations and button mappings
 *          a universal processing method cannot be implemented. Users should acquire data from 
 *          their specific remote controller, normalize it as needed, and implement custom
 *          interaction logic based on the application requirements.
 */
void RcMsgHandler::masterHandle()
{
#if EXTENSION_DT7 == 1
    static DT7::RcData_s rcDataPrev = {};
    static constexpr float CH_RANGE = DT7::CH_VALUE_MAX - DT7::CH_VALUE_OFFSET;
    static constexpr float WHEEL_THESHORLD = 0.2f;
    using SW = DT7::Sw_e;
    DT7::RcData_s *rcData = static_cast<DT7::RcData_s *>(rc->getData());
    SW &sr = rcData->rc.switchRight;
    SW &sl = rcData->rc.switchLeft;
    if (sr == SW::DOWN) {
        gMsg_.state = GIMBAL::FSMState_e::STOP;
        gMsg_.wPitch = gMsg_.wYaw = 0.f;
        cMsg_.state = CHASSIS::FSMState_e::STOP;
        cMsg_.vx = cMsg_.vy = 0.f;
        alMsg_.state = ARMORLOADER::FSMState_e::STOP;
        alMsg_.fire = false;
        abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
    } else if (sr == SW::MID) {
        cMsg_.vx = static_cast<float>(rcData->rc.ch1) / CH_RANGE * INPUT_VX_MAX;
        cMsg_.vy = -static_cast<float>(rcData->rc.ch0) / CH_RANGE * INPUT_VY_MAX;
        gMsg_.wPitch = -static_cast<float>(rcData->rc.ch3) / CH_RANGE * INPUT_PITCH_MAX;
        gMsg_.wYaw = -static_cast<float>(rcData->rc.ch2) / CH_RANGE * INPUT_YAW_MAX;
        alMsg_.fire = std::fabs(static_cast<float>(rcData->wheel - DT7::CH_VALUE_OFFSET) / CH_RANGE) > WHEEL_THESHORLD;
        if (sl == SW::DOWN) {
            gMsg_.state = GIMBAL::FSMState_e::RUN;
            cMsg_.state = CHASSIS::FSMState_e::IDLE;
            alMsg_.state = ARMORLOADER::FSMState_e::STOP;
            abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
        } else if (sl == SW::MID) {
            gMsg_.state = GIMBAL::FSMState_e::AUTO;
            cMsg_.state = CHASSIS::FSMState_e::TANK;
            alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
            abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
        } else if (sl == SW::UP) {
            alMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
        }
    } else if (sr == SW::UP) {
        cMsg_.vx = static_cast<float>(rcData->rc.ch1) / CH_RANGE * INPUT_VX_MAX;
        cMsg_.vy = -static_cast<float>(rcData->rc.ch0) / CH_RANGE * INPUT_VY_MAX;
        gMsg_.wPitch = -static_cast<float>(rcData->rc.ch3) / CH_RANGE * INPUT_PITCH_MAX;
        gMsg_.wYaw = -static_cast<float>(rcData->rc.ch2) / CH_RANGE * INPUT_YAW_MAX;
        alMsg_.fire = false;
        if (sl == SW::DOWN) {
            cMsg_.state = CHASSIS::FSMState_e::TANK;
        } else {
            cMsg_.state = CHASSIS::FSMState_e::SPIN;
        }
        gMsg_.state = GIMBAL::FSMState_e::RUN;
        alMsg_.state = ARMORLOADER::FSMState_e::STOP;
        abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
    } else {
        // TODO: Use keyboard
        return;
    }
    memcpy(&rcDataPrev, rcData, sizeof(DT7::RcData_s));
#elif EXTENSION_ET08A == 1
    static ET08A::RcData_s rcDataPrev = {};
    static constexpr float CH_RANGE = ET08A::CH_VALUE_MAX - ET08A::CH_VALUE_OFFSET;
    static constexpr float WHEEL_THESHORLD = 0.2f;
    using SW = ET08A::Sw_e;
    ET08A::RcData_s *rcData = static_cast<ET08A::RcData_s *>(rc->getData());
    SW &sr = rcData->switchRight;
    SW &sl = rcData->switchLeft;
    if (sr == SW::DOWN) {
        gMsg_.state = GIMBAL::FSMState_e::STOP;
        gMsg_.wPitch = gMsg_.wYaw = 0.f;
        cMsg_.state = CHASSIS::FSMState_e::STOP;
        cMsg_.vx = cMsg_.vy = 0.f;
        alMsg_.state = ARMORLOADER::FSMState_e::STOP;
        alMsg_.fire = false;
        abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
    } else if (sr == SW::MID) {
        cMsg_.vx = static_cast<float>(rcData->ch1) / CH_RANGE * INPUT_VX_MAX;
        cMsg_.vy = -static_cast<float>(rcData->ch0) / CH_RANGE * INPUT_VY_MAX;
        gMsg_.wPitch = -static_cast<float>(rcData->ch3) / CH_RANGE * INPUT_PITCH_MAX;
        gMsg_.wYaw = -static_cast<float>(rcData->ch2) / CH_RANGE * INPUT_YAW_MAX;
        alMsg_.fire = std::fabs(static_cast<float>(rcData->wheel - ET08A::CH_VALUE_OFFSET) / CH_RANGE) >
                      WHEEL_THESHORLD;
        if (sl == SW::DOWN) {
            gMsg_.state = GIMBAL::FSMState_e::RUN;
            cMsg_.state = CHASSIS::FSMState_e::IDLE;
            alMsg_.state = ARMORLOADER::FSMState_e::STOP;
            abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
        } else if (sl == SW::MID) {
            gMsg_.state = GIMBAL::FSMState_e::AUTO;
            cMsg_.state = CHASSIS::FSMState_e::TANK;
            alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
            abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
        } else if (sl == SW::UP) {
            alMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
        }
    } else if (sr == SW::UP) {
        cMsg_.vx = static_cast<float>(rcData->ch1) / CH_RANGE * INPUT_VX_MAX;
        cMsg_.vy = -static_cast<float>(rcData->ch0) / CH_RANGE * INPUT_VY_MAX;
        gMsg_.wPitch = -static_cast<float>(rcData->ch3) / CH_RANGE * INPUT_PITCH_MAX;
        gMsg_.wYaw = -static_cast<float>(rcData->ch2) / CH_RANGE * INPUT_YAW_MAX;
        alMsg_.fire = false;
        if (sl == SW::DOWN) {
            cMsg_.state = CHASSIS::FSMState_e::TANK;
        } else {
            cMsg_.state = CHASSIS::FSMState_e::SPIN;
        }
        gMsg_.state = GIMBAL::FSMState_e::RUN;
        alMsg_.state = ARMORLOADER::FSMState_e::STOP;
        abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
    } else {
        return;
    }
#elif EXTENSION_VT13 == 1
    // TODO: VT13
#endif
    notify(&gMsg_, msgBus_->gimbalQueue);
    notify(&cMsg_, msgBus_->chassisQueue);
    notify(&alMsg_, msgBus_->armorLoaderQueue);
    notify(&abMsg_, msgBus_->armorBoosterQueue);
}

void RcMsgHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }
