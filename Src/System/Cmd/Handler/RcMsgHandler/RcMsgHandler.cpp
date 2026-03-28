#include "RcMsgHandler.hpp"
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

#include "magic_enum/magic_enum.hpp"

#include "Gimbal.hpp"
#include "Shooter.hpp"

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
    } else if constexpr (DEV_TYPE == RCDevType_e::ET08A) {
        rc_ = std::make_unique<ET08A>(_huart, _event, this->bit_);
    } else if constexpr (DEV_TYPE == RCDevType_e::VT13) {
        rc_ = std::make_unique<VT13>(_huart, _event, this->bit_);
    }
    scheduleDaemon();
}

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::scheduleDaemon()
{
#if APP_USE_DAEMONS
    Daemons::instance().schedule([this]() {
        static uint32_t updateCnt = 0;
        if (!this->rc_->isOnline()) {
            if (xTaskGetTickCount() - updateCnt >= 2000) {
                updateCnt = xTaskGetTickCount();
                LOG::warn("RcMsgHandler", "%s offline", magic_enum::enum_name(DEV_TYPE).data());
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

void updateRocker(float &_target, float _channel)
{
    float S_CURVE_ACC = 2.0f;
    uint8_t T_ACC_CNT = 100;
    float delta = (_channel * T_ACC_CNT / 660.0f) - _target;
    _target += std::fmax(-S_CURVE_ACC, std::fmin(S_CURVE_ACC, delta));
}

void getTurnWheelId(int8_t& Id, float _rx){
    static bool state = false;
    static bool lstate = false;

    if(fabsf(_rx) > 50.f){
        state = true;
    }
    else{
        state = false;
    }

    if(state == true && lstate == false){
        if(_rx > 0){Id++;}
        else if(_rx < 0){Id--;} 
    }
    else{;}

    if(Id > 3){Id = 0;}
    else if(Id < 0){Id = 3;}
    else{;}

    lstate = state;
}

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
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
        updateRocker(rcMsg_.rx, (float)rcData->rc.ch0);
        updateRocker(rcMsg_.ry, (float)rcData->rc.ch1);
        updateRocker(rcMsg_.lx, (float)rcData->rc.ch2);
        updateRocker(rcMsg_.ly, (float)rcData->rc.ch3);
        rcMsg_.wheel = (float)rcData->wheel;

        rcMsg_.rSwitch = rcData->rc.switchRight;
        rcMsg_.lSwitch = rcData->rc.switchLeft;
    } else if constexpr (DEV_TYPE == RCDevType_e::ET08A) {
        ET08A::RcData_s *rcData = static_cast<ET08A::RcData_s *>(rc_->getData());
        (void)rcData;
    } else if constexpr (DEV_TYPE == RCDevType_e::VT13) {
        VT13::RcData_s *rcData = static_cast<VT13::RcData_s *>(rc_->getData());
        (void)rcData;
    }
    static DT7::Sw_e rswitchLast = DT7::Sw_e::DOWN;
    static DT7::Sw_e lswitchLast = DT7::Sw_e::DOWN;

    static float ratioYaw = (1.f / 100 / 1000);//1s最多转动1度
    static float ratioScrew = 0.00045f;
    static float ratioTrig = 0.000025f;

    switch(rcMsg_.rSwitch){
        case DT7::Sw_e::DOWN:{
            gmsg_.state = GIMBAL::FSMState_e::STOP;
            smsg_.state = SHOOTER::FSMState_e::STOP;
            smsg_.turnId = 0;
            break;
        }
        case DT7::Sw_e::MID:{
            gmsg_.state = GIMBAL::FSMState_e::RUN;
            smsg_.state = SHOOTER::FSMState_e::RUN;

            gmsg_.isConstant = false;
            smsg_.isConstant = false;

            if(rcMsg_.lSwitch == DT7::Sw_e::UP){
                gmsg_.visFollow = true;
            }
            else{
                gmsg_.visFollow = false;
            }

            if(rcMsg_.lSwitch == DT7::Sw_e::MID && lswitchLast == DT7::Sw_e::DOWN){
                gmsg_.state  = GIMBAL::FSMState_e::LAUNCH;
                smsg_.state = SHOOTER::FSMState_e::LAUNCH;
            }
            else{;}
//yaw
            if(fabsf(rcMsg_.lx) > 50.f)
                gmsg_.gainYaw = (rcMsg_.lx * ratioYaw);
            else
                gmsg_.gainYaw = 0;
//trigger
            if(fabsf(rcMsg_.ly) > 50.f)
                gmsg_.gainTrig = (rcMsg_.ly * ratioTrig);
            else
                gmsg_.gainTrig = 0;
//turnWheel
            getTurnWheelId(smsg_.turnId, rcMsg_.rx);
//screw
            if(fabsf(rcMsg_.ry) > 50.f)
                smsg_.gainScrew = (rcMsg_.ry * ratioScrew);
            else
                smsg_.gainScrew = 0;
//fireFlag
            if(fabsf(rcMsg_.wheel - 1024.f) > 400.f)
                smsg_.fireFlag = true;
            else
                smsg_.fireFlag = false;

            break;
        }
        case DT7::Sw_e::UP:{
            gmsg_.isConstant = true;
            smsg_.isConstant = true;
            break;
        }
    }

    lswitchLast = rcMsg_.lSwitch;
    rswitchLast = rcMsg_.rSwitch;

    notify(&gmsg_, msgBus_->gimbalQueue);
    notify(&smsg_, msgBus_->shooterQueue);
    // User-defined processing logic goes here ↑
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
