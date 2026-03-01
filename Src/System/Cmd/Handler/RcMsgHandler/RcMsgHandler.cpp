#include "RcMsgHandler.hpp"
#include "sdkconfig.h"

#include "RemoteControl.hpp"

#include "Chassis.hpp"

#ifdef GIMBAL_TYPE
#include GIMBAL_FILE
#endif
#if APP_USE_ARM
#include "Arm.hpp"
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
#if APP_USE_GIMBAL == 1
template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::masterHandle()
{
    ChassisMsg_s chassisMsg{};
    GimbalMsg_s gimbalMsg{};
    ArmMsg_s armMsg{};

    // User-defined processing logic goes here ↓
    if constexpr (DEV_TYPE == RCDevType_e::DT7) {
        using SW = DT7::Sw_e;
        static constexpr float CH_RANGE = DT7::CH_VALUE_MAX - DT7::CH_VALUE_OFFSET;
        DT7::RcData_s *rcData = static_cast<DT7::RcData_s *>(rc_->getData());

        const float &rx = rcData->rc.ch0;
        const float &ry = rcData->rc.ch1;
        const float &lx = rcData->rc.ch2;
        const float &ly = rcData->rc.ch3;

#if APP_USE_KEY
        /*
         * gimbal:STOP,RUN,
         * chassis:STOP,RUN,SPIN
         * arm:ONECLICK,MODE,STOP,NORMAL,PLAN,TEACH,夹爪
         */
        //机械臂控制使用自定义控制器
        /*————————————————————————————————————————————————chassis——————————————————————————————————————————————*/
        float accK = 1.f, lxCommand = 0;
        chassisMsg.state = CHASSIS::FSMState_e::RUN;
        //加速
        if (isKeyPressed(KEY_SHIFT))
            accK = 1.5f;
        else
            accK = 1.f;

        //前后左右
        if (isKeyPressed(KEY_W)) {
            kupdateRocker(rcMsg_.ry, 1); //加速向前，ry 增加并趋近于最大值。
        } else if (isKeyPressed(KEY_S)) {
            kupdateRocker(rcMsg_.ry, -1); //加速向后，ry 减少并趋近于最小值。
        } else {
            kupdateRocker(rcMsg_.ry, 0); //自然减速回中，ry 向 0 值靠近。
        }

        if (isKeyPressed(KEY_A)) {
            kupdateRocker(rcMsg_.rx, -1);
        } else if (isKeyPressed(KEY_D)) {
            kupdateRocker(rcMsg_.rx, 1);
        } else {
            kupdateRocker(rcMsg_.rx, 0);
        }

        //旋转
        if (rcMsg_.rPress == 1)
            lxCommand = 1;
        else if (rcMsg_.lPress == 1)
            lxCommand = -1;
        else
            lxCommand = 0;

        kupdateRocker(rcMsg_.lx, lxCommand);

        chassisMsg.vx = SMOOTH::sCurve(accK * Chassis::MAX_VX_SPEED, rcMsg_.ry, T_ACC_CNT);  // Scale to m/s
        chassisMsg.vy = SMOOTH::sCurve(accK * Chassis::MAX_VY_SPEED, -rcMsg_.rx, T_ACC_CNT); // Scale to m/s
        chassisMsg.yaw = SMOOTH::sCurve(accK * Chassis::MAX_WZ_SPEED, rcMsg_.lx, T_ACC_CNT); // Scale to m/s
        /*——————————————————————————————————————————————————————————————————————————————————————————————————————————*/
        //单双臂
        if (isKeyPressed(KEY_SHIFT) && !isKeyLastPressed(KEY_SHIFT)) {
            if (isKeyLastPressed(KEY_Z) && !isKeyPressed(KEY_Z)) {
                armMsg.mode = ARM::ArmMode_e::LEFT;
            }
            if (isKeyLastPressed(KEY_X) && !isKeyPressed(KEY_X)) {
                armMsg.mode = ARM::ArmMode_e::RIGHT;
            }
            if (isKeyLastPressed(KEY_C) && !isKeyPressed(KEY_C)) {
                armMsg.mode = ARM::ArmMode_e::TWO;
            }
        }
        //一键: 取矿*3，从矿仓取矿，公路区跨越, 夹爪
        if (isKeyPressed(KEY_Q))
            armMsg.oneClick = ARM::OneClick_e::ORE_PICK1;
        if (isKeyPressed(KEY_E))
            armMsg.oneClick = ARM::OneClick_e::ORE_PICK2;
        if (isKeyPressed(KEY_R))
            armMsg.oneClick = ARM::OneClick_e::ORE_PICK3;
        if (isKeyPressed(KEY_F))
            armMsg.oneClick = ARM::OneClick_e::HIGHWAY_CROSS;
        if (isKeyPressed(KEY_G))
            armMsg.oneClick = ARM::OneClick_e::ORE_BIN_EXTRACT;
        if (isKeyPressed(KEY_V) && !isKeyLastPressed(KEY_V))
            armMsg.switch1 = !armMsg.switch1;
#else
        if (rcData->rc.switchRight == SW::DOWN) {
            //arm
            armMsg.switch1 = false;
            armMsg.state = ARM::FSMState_e::STOP;
            // gimbal
            gimbalMsg.state = GIMBAL::FSMState_e::STOP;
            gimbalMsg.pitch = gimbalMsg.yaw = 0.f;
            // chassis
            chassisMsg.state = CHASSIS::FSMState_e::STOP;
            chassisMsg.vx = chassisMsg.vy = 0.f;
        } else if (rcData->rc.switchRight == SW::MID) {
            if (rcData->rc.switchLeft == SW::DOWN) {
                // gimbal
                gimbalMsg.state = GIMBAL::FSMState_e::RUN;
                gimbalMsg.yaw = -lx / CH_RANGE * INPUT_YAW_MAX;
                // chassis
                chassisMsg.state = CHASSIS::FSMState_e::RUN;
                chassisMsg.vx = ry / CH_RANGE * INPUT_VX_MAX;
                chassisMsg.vy = -rx / CH_RANGE * INPUT_VY_MAX;

            } else if (rcData->rc.switchLeft == SW::MID) {
                //arm
                armMsg.target[0] = -lx / 314 / 200;
                armMsg.target[1] = ly / 314 / 100;
                //夹爪
                if (rcData->rc.ch1 == 660)
                    armMsg.switch1 = !armMsg.switch1;
            } else if (rcData->rc.switchLeft == SW::UP) {
                //arm
                armMsg.target[2] = ly / 314 / 150;  //pitch
                armMsg.target[3] = lx / 314 / 50;   //yaw
                armMsg.target[4] = -ry / 314 / 100; //pitch
                armMsg.target[5] = rx / 314 / 50;   //roll

                armMsg.target[6] = ly / 314 / 150;  //pitch
                armMsg.target[7] = -lx / 314 / 50;  //yaw
                armMsg.target[8] = -ry / 314 / 100; //pitch
                armMsg.target[9] = -rx / 314 / 50;  //roll
            }
        } else if (rcData->rc.switchRight == SW::UP) {
            // gimbal
            gimbalMsg.state = GIMBAL::FSMState_e::RUN;
            gimbalMsg.yaw = -lx / CH_RANGE * INPUT_YAW_MAX;
            // chassis
            if (rcData->rc.switchLeft == SW::DOWN) {
            } else {
                chassisMsg.state = CHASSIS::FSMState_e::SPIN;
            }
            chassisMsg.vx = ry / CH_RANGE * INPUT_VX_MAX;
            chassisMsg.vy = -rx / CH_RANGE * INPUT_VY_MAX;
        } else {
            return;
        }

#endif
    } else if constexpr (DEV_TYPE == RCDevType_e::ET08A) {
        ET08A::RcData_s *rcData = static_cast<ET08A::RcData_s *>(rc_->getData());
        (void)rcData;
    } else if constexpr (DEV_TYPE == RCDevType_e::VT13) {
        VT13::RcData_s *rcData = static_cast<VT13::RcData_s *>(rc_->getData());
        (void)rcData;
    }

    notify(&chassisMsg, msgBus_->chassisQueue);
    notify(&gimbalMsg, msgBus_->gimbalQueue);
    notify(&armMsg, msgBus_->armQueue);
}


#else
template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::masterHandle()
{
    ChassisMsg_s chassisMsg{};
    if (rcMsg_.rSwitch == RC_SW_DOWN) {
        chassisMsg.state = CHASSIS::FSMState_e::STOP;
        chassisMsg.vx = 0.f;
        chassisMsg.vy = 0.f;
        chassisMsg.yaw = 0.f;
    } else {
        if (rcMsg_.lSwitch == RC_SW_DOWN) {
            chassisMsg.state = CHASSIS::FSMState_e::RUN;
        } else {
            chassisMsg.state = CHASSIS::FSMState_e::SPIN;
        }
        chassisMsg.vx = SMOOTH::sCurve(ROCKER_VX_GAIN, rcMsg_.ry, T_ACC_CNT);
        chassisMsg.vy = -SMOOTH::sCurve(ROCKER_VY_GAIN, rcMsg_.rx, T_ACC_CNT);
        chassisMsg.yaw = SMOOTH::sCurve(ROCKER_YAW_GAIN, rcMsg_.lx, T_ACC_CNT);
    }
    memcpy(&rcMsgPrev_, &rcMsg_, sizeof(rcMsg_));
    notify(&chassisMsg, msgBus_->chassisQueue);
}
#endif

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
