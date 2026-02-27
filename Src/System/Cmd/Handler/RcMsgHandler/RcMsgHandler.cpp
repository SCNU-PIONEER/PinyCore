#include "RcMsgHandler.hpp"
#include "ArmorBooster.hpp"
#include "ArmorLoader.hpp"
#include "sdkconfig.h"
#include "RemoteControl.hpp"

#ifdef CHASSIS_TYPE
#include CHASSIS_FILE
#endif
#ifdef GIMBAL_TYPE
#include GIMBAL_FILE
#endif

#include "DT7.hpp"
#include "ET08A.hpp"
#include "VT13.hpp"

#include "Daemons.hpp"
#include "StmLog.hpp"

#include <cmath>
#include <cstring>

template <RCDevType_e DEV_TYPE>
RcMsgHandler<DEV_TYPE>::RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event)
{
    createRcDev(_huart, _event);
};

template <RCDevType_e DEV_TYPE>
void RcMsgHandler<DEV_TYPE>::createRcDev(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event)
{
    if constexpr (DEV_TYPE == RCDevType_e::DT7) {
        rc_ = std::make_unique<DT7>(_huart, _event, this->bit_);
        scheduleDaemon("DT7 offline");
    } else if constexpr (DEV_TYPE == RCDevType_e::ET08A) {
        rc_ = std::make_unique<ET08A>(_huart, _event, this->bit_);
        scheduleDaemon("ET08A offline");
    } else if constexpr (DEV_TYPE == RCDevType_e::VT13) {
        rc_ = std::make_unique<VT13>(_huart, _event, this->bit_);
        scheduleDaemon("VT13 offline");
    }
}

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::scheduleDaemon(const char *_format)
{
#if APP_USE_DAEMONS
    Daemons::instance().schedule([this, _format]() {
        static uint32_t updateCnt = 0;
        if (!this->rc_->isOnline()) {
            if (xTaskGetTickCount() - updateCnt >= 2000) {
                updateCnt = xTaskGetTickCount();
                LOG::warn("Daemons", _format);
            }
        }
    });
#endif
}

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
}

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::handle()
{
    rc_->parse();

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
template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::masterHandle()
{
    // User-defined processing logic goes here ↓
    if constexpr (DEV_TYPE == RCDevType_e::DT7) {
        DT7::RcData_s *rcData = static_cast<DT7::RcData_s *>(rc_->getData());
        static DT7::RcData_s rcDataPrev = {};
        static constexpr float CH_RANGE = DT7::CH_VALUE_MAX - DT7::CH_VALUE_OFFSET;
        static constexpr float WHEEL_THESHORLD = 0.2f;
        using SW = DT7::Sw_e;
        SW &sr = rcData->rc.switchRight;
        SW &sl = rcData->rc.switchLeft;
        float wheelRocker = ((float)rcData->wheel - 1024.f) / 660.f;
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
            alMsg_.fire = std::fabs(static_cast<float>(rcData->wheel - DT7::CH_VALUE_OFFSET) / CH_RANGE) >
                          WHEEL_THESHORLD;
            if (sl == SW::DOWN) {
                gMsg_.state = GIMBAL::FSMState_e::RUN;
                cMsg_.state = CHASSIS::FSMState_e::RUN;
                alMsg_.state = ARMORLOADER::FSMState_e::STOP;
                abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
            } else if (sl == SW::MID) {
                gMsg_.state = GIMBAL::FSMState_e::RUN;
                cMsg_.state = CHASSIS::FSMState_e::TANK;
                alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
                abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
            } else if (sl == SW::UP) {
                alMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
            }
            // cMsg_.state = CHASSIS::FSMState_e::STOP;

        } else if (sr == SW::UP) {
            cMsg_.vx = static_cast<float>(rcData->rc.ch1) / CH_RANGE * INPUT_VX_MAX;
            cMsg_.vy = -static_cast<float>(rcData->rc.ch0) / CH_RANGE * INPUT_VY_MAX;
            gMsg_.wPitch = -static_cast<float>(rcData->rc.ch3) / CH_RANGE * INPUT_PITCH_MAX;
            gMsg_.wYaw = -static_cast<float>(rcData->rc.ch2) / CH_RANGE * INPUT_YAW_MAX;
            // alMsg_.fire = std::fabs(static_cast<float>(rcData->wheel - DT7::CH_VALUE_OFFSET) / CH_RANGE) > WHEEL_THESHORLD;
            alMsg_.fire = wheelRocker > WHEEL_THESHORLD || wheelRocker < -WHEEL_THESHORLD;
            if (sl == SW::DOWN) {
                // gMsg_.state = GIMBAL::FSMState_e::NAV;
                // cMsg_.state = CHASSIS::FSMState_e::TANK;
                gMsg_.state = GIMBAL::FSMState_e::PATROL;
                cMsg_.state = CHASSIS::FSMState_e::SPIN;
            } else if (sl == SW::MID) {
                gMsg_.state = GIMBAL::FSMState_e::AUTO;
                alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
                abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
            } else if (sl == SW::UP) {
                alMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
            }
            // cMsg_.state = CHASSIS::FSMState_e::STOP;
        } else {
            // TODO: Use keyboard
            return;
        }
        memcpy(&rcDataPrev, rcData, sizeof(DT7::RcData_s));
    } else if constexpr (DEV_TYPE == RCDevType_e::ET08A) {
        ET08A::RcData_s *rcData = static_cast<ET08A::RcData_s *>(rc_->getData());
        static ET08A::RcData_s rcDataPrev = {};
        static constexpr float CH_RANGE = ET08A::CH_VALUE_MAX - ET08A::CH_VALUE_OFFSET;
        static constexpr float WHEEL_THESHORLD = 0.2f;
        using SW = ET08A::Sw_e;
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
                cMsg_.state = CHASSIS::FSMState_e::RUN;
                alMsg_.state = ARMORLOADER::FSMState_e::STOP;
                abMsg_.state = ARMORBOOSTER::FSMState_e::OFF;
            } else if (sl == SW::MID) {
                gMsg_.state = GIMBAL::FSMState_e::RUN;
                cMsg_.state = CHASSIS::FSMState_e::TANK;
                alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
                abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
            } else if (sl == SW::UP) {
                alMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
            }
            cMsg_.state = CHASSIS::FSMState_e::STOP;
        } else if (sr == SW::UP) {
            cMsg_.vx = static_cast<float>(rcData->ch1) / CH_RANGE * INPUT_VX_MAX;
            cMsg_.vy = -static_cast<float>(rcData->ch0) / CH_RANGE * INPUT_VY_MAX;
            gMsg_.wPitch = -static_cast<float>(rcData->ch3) / CH_RANGE * INPUT_PITCH_MAX;
            gMsg_.wYaw = -static_cast<float>(rcData->ch2) / CH_RANGE * INPUT_YAW_MAX;
            alMsg_.fire = std::fabs(static_cast<float>(rcData->wheel - ET08A::CH_VALUE_OFFSET) / CH_RANGE) >
                          WHEEL_THESHORLD;
            if (sl == SW::DOWN) {
                // gMsg_.state = GIMBAL::FSMState_e::NAV;
                cMsg_.state = CHASSIS::FSMState_e::TANK;
                // gMsg_.state = GIMBAL::FSMState_e::PATROL;
                // cMsg_.state = CHASSIS::FSMState_e::SPIN;
            } else if (sl == SW::MID) {
                gMsg_.state = GIMBAL::FSMState_e::AUTO;
                // alMsg_.state = ARMORLOADER::FSMState_e::AUTO;
                // abMsg_.state = ARMORBOOSTER::FSMState_e::ON;
            } else if (sl == SW::UP) {
                // alMsg_.state = ARMORLOADER::FSMState_e::SINGLE;
            }

            cMsg_.state = CHASSIS::FSMState_e::STOP;
        } else if constexpr (DEV_TYPE == RCDevType_e::VT13) {
            VT13::RcData_s *rcData = static_cast<VT13::RcData_s *>(rc_->getData());
            (void)rcData;
        }
        // User-defined processing logic goes here ↑
    }
    notify(&gMsg_, msgBus_->gimbalQueue);
    notify(&cMsg_, msgBus_->chassisQueue);
    notify(&alMsg_, msgBus_->armorLoaderQueue);
    notify(&abMsg_, msgBus_->armorBoosterQueue);
}
template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}

// Explicit template instantiation
#if EXTENSION_DT7
template class RcMsgHandler<RCDevType_e::DT7>;
#elif EXTENSION_ET08A
template class RcMsgHandler<RCDevType_e::ET08A>;
#endif
#if EXTENSION_VT13
template class RcMsgHandler<RCDevType_e::VT13>;
#endif
