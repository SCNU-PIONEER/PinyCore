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
#include "Smooth.hpp"

#include "magic_enum/magic_enum.hpp"

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
        channelMax_ = 660.f;
    } else if constexpr (DEV_TYPE == RCDevType_e::ET08A) {
        rc_ = std::make_unique<ET08A>(_huart, _event, this->bit_);
        channelMax_ = 1.f; // 未测
    } else if constexpr (DEV_TYPE == RCDevType_e::VT13) {
        rc_ = std::make_unique<VT13>(_huart, _event, this->bit_);
        channelMax_ = 1.f;
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
    static ArmMsg_s armMsg{};

    // User-defined processing logic goes here ↓
    if constexpr (DEV_TYPE == RCDevType_e::DT7) {
        using SW = DT7::Sw_e;
        static constexpr float CH_RANGE = DT7::CH_VALUE_MAX - DT7::CH_VALUE_OFFSET;
        DT7::RcData_s *rcData = static_cast<DT7::RcData_s *>(rc_->getData());

        updateRocker(rx, rcData->rc.ch0);
        updateRocker(ry, rcData->rc.ch1);
        updateRocker(lx, rcData->rc.ch2);
        updateRocker(ly, rcData->rc.ch3);

        gimbalVal = -((float)rcData->rc.ch2 * GIMBAL_YAW_MAX / 660);


#if APP_USE_DT7_KEYBOARD
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
        if (IS_KEY_DT7(SHIFT))
            accK = 1.5f;
        else
            accK = 1.f;

        //前后左右
        if (IS_KEY_DT7(W)) {
            updateRocker(ry, 1); //加速向前，ry 增加并趋近于最大值。
        } else if (IS_KEY_DT7(S)) {
            updateRocker(ry, -1); //加速向后，ry 减少并趋近于最小值。
        } else {
            updateRocker(ry, 0); //自然减速回中，ry 向 0 值靠近。
        }

        if (IS_KEY_DT7(A)) {
            updateRocker(rx, -1);
        } else if (IS_KEY_DT7(D)) {
            updateRocker(rx, 1);
        } else {
            updateRocker(rx, 0);
        }

        //旋转
        if (rcData->mouse.pressRight == 1)
            lxCommand = 1;
        else if (rcData->mouse.pressLeft == 1)
            lxCommand = -1;
        else
            lxCommand = 0;

        updateRocker(lx, lxCommand);

        chassisMsg.vx = SMOOTH::sCurve(accK * Chassis::MAX_VX_SPEED, ry, T_ACC_CNT);  // Scale to m/s
        chassisMsg.vy = SMOOTH::sCurve(accK * Chassis::MAX_VY_SPEED, -rx, T_ACC_CNT); // Scale to m/s
        chassisMsg.yaw = SMOOTH::sCurve(accK * Chassis::MAX_WZ_SPEED, lx, T_ACC_CNT); // Scale to m/s
        /*——————————————————————————————————————————————————————————————————————————————————————————————————————————*/
        //单双臂
        if (IS_KEY_DT7(SHIFT) && !IS_LAST_KEY_DT7(SHIFT)) {
            if (IS_LAST_KEY_DT7(Z) && !IS_KEY_DT7(Z)) {
                armMsg.mode = ARM::ArmMode_e::LEFT;
            }
            if (IS_LAST_KEY_DT7(X) && !IS_KEY_DT7(X)) {
                armMsg.mode = ARM::ArmMode_e::RIGHT;
            }
            if (IS_LAST_KEY_DT7(C) && !IS_KEY_DT7(C)) {
                armMsg.mode = ARM::ArmMode_e::TWO;
            }
        }
        //一键: 取矿*3，从矿仓取矿，公路区跨越, 夹爪
        if (IS_KEY_DT7(Q))
            armMsg.oneClick = ARM::OneClick_e::ORE_PICK1;
        if (IS_KEY_DT7(E))
            armMsg.oneClick = ARM::OneClick_e::ORE_PICK2;
        if (IS_KEY_DT7(R))
            armMsg.oneClick = ARM::OneClick_e::ORE_PICK3;
        if (IS_KEY_DT7(F))
            armMsg.oneClick = ARM::OneClick_e::HIGHWAY_CROSS;
        if (IS_KEY_DT7(G))
            armMsg.oneClick = ARM::OneClick_e::ORE_BIN_EXTRACT;
        if (IS_KEY_DT7(V) && !IS_LAST_KEY_DT7(V))
            armMsg.switch1 = !armMsg.switch1;
#endif
        //拨轮控夹爪
        if (rcData->wheel == 1684)
            armMsg.switch1 = !armMsg.switch1;
        if (rcData->rc.switchRight == SW::DOWN) {
            //arm
            armMsg.switch1 = false;
            armMsg.state = ARM::FSMState_e::STOP;
            // gimbal
            gimbalMsg.state = GIMBAL::FSMState_e::STOP;
            gimbalMsg.yaw = 0.f;
            // chassis
            chassisMsg.state = CHASSIS::FSMState_e::STOP;
            chassisMsg.vx = chassisMsg.vy = 0.f;
        } else if (rcData->rc.switchRight == SW::MID) {
            if (rcData->rc.switchLeft == SW::DOWN) {
                // gimbal
                gimbalMsg.state = GIMBAL::FSMState_e::RUN;
                gimbalMsg.yaw = gimbalVal;
                if (rcData->rc.ch1 == 660) { //ry
                    chassisMsg.state = CHASSIS::FSMState_e::SPIN;
                } else if (rcData->rc.ch1 == -660) { //ry
                    // chassis
                    chassisMsg.state = CHASSIS::FSMState_e::RUN;
                }
                chassisMsg.vx = ry / 100 * INPUT_VX_MAX;
                chassisMsg.vy = -rx / 100 * INPUT_VY_MAX;
            } else if (rcData->rc.switchLeft == SW::MID) {
                //arm
                armMsg.state = ARM::FSMState_e::NORMAL;
                armMsg.target[0] = lx / 314 / 50; //8016yaw
                armMsg.target[1] = ly / 314 / 50; //10010pitch
                if (armMsg.mode == ARM::ArmMode_e::LEFT) {
                    armMsg.target[2] = rx / 314 / 50; //8009pitch
                    armMsg.target[3] = ry / 314 / 50; //4340yaw
                } else if (armMsg.mode == ARM::ArmMode_e::RIGHT) {
                    armMsg.target[6] = rx / 314 / 50; //8009pitch
                    armMsg.target[7] = ry / 314 / 50; //4340yaw
                }
            } else if (rcData->rc.switchLeft == SW::UP) {
                //arm
                armMsg.state = ARM::FSMState_e::NORMAL;
                if (armMsg.mode == ARM::ArmMode_e::LEFT) {
                    armMsg.target[4] = lx / 314 / 50; //3507roll
                    armMsg.target[5] = ly / 314 / 50; //3507pitch
                } else if (armMsg.mode == ARM::ArmMode_e::RIGHT) {
                    armMsg.target[8] = lx / 314 / 50; //3507roll
                    armMsg.target[9] = ly / 314 / 50; //3507pitch
                }
                //左右臂切换
                if (rcData->rc.ch0 == -660) //rx
                    armMsg.mode = ARM::ArmMode_e::LEFT;
                else if (rcData->rc.ch0 == 660) //rx
                    armMsg.mode = ARM::ArmMode_e::RIGHT;
            }
        } else if (rcData->rc.switchRight == SW::UP) {
        }

    } else if constexpr (DEV_TYPE == RCDevType_e::ET08A) {
        ET08A::RcData_s *rcData = static_cast<ET08A::RcData_s *>(rc_->getData());
        (void)rcData;
    } else if constexpr (DEV_TYPE == RCDevType_e::VT13) {
        VT13::RcData_s *rcData = static_cast<VT13::RcData_s *>(rc_->getData());
        (void)rcData;
        updateRocker(rx, rcData->gamepad.ch[0]);
        updateRocker(ry, rcData->gamepad.ch[1]);
        updateRocker(lx, rcData->gamepad.ch[2]);
        updateRocker(ly, rcData->gamepad.ch[3]);
#if APP_USE_VT13_KEYBOARD
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
        if (IS_KEY_VT13(SHIFT))
            accK = 1.5f;
        else
            accK = 1.f;

        //前后左右
        if (IS_KEY_VT13(W)) {
            updateRocker(ry, 1); //加速向前，ry 增加并趋近于最大值。
        } else if (IS_KEY_VT13(S)) {
            updateRocker(ry, -1); //加速向后，ry 减少并趋近于最小值。
        } else {
            updateRocker(ry, 0); //自然减速回中，ry 向 0 值靠近。
        }

        if (IS_KEY_VT13(A)) {
            updateRocker(rx, -1);
        } else if (IS_KEY_VT13(D)) {
            updateRocker(rx, 1);
        } else {
            updateRocker(rx, 0);
        }

        //旋转
        if (rcData->KBM.mouseRight == 1)
            lxCommand = 1;
        else if (rcData->KBM.mouseLeft == 1)
            lxCommand = -1;
        else
            lxCommand = 0;

        updateRocker(lx, lxCommand);

        chassisMsg.vx = SMOOTH::sCurve(accK * Chassis::MAX_VX_SPEED, ry, T_ACC_CNT);  // Scale to m/s
        chassisMsg.vy = SMOOTH::sCurve(accK * Chassis::MAX_VY_SPEED, -rx, T_ACC_CNT); // Scale to m/s
        chassisMsg.yaw = SMOOTH::sCurve(accK * Chassis::MAX_WZ_SPEED, lx, T_ACC_CNT); // Scale to m/s
        /*——————————————————————————————————————————————————————————————————————————————————————————————————————————*/
        //单双臂
        if (IS_KEY_VT13(SHIFT) && !IS_LAST_KEY_VT13(SHIFT)) {
            if (IS_LAST_KEY_VT13(Z) && !IS_KEY_VT13(Z)) {
                armMsg.mode = ARM::ArmMode_e::LEFT;
            }
            if (IS_LAST_KEY_VT13(X) && !IS_KEY_VT13(X)) {
                armMsg.mode = ARM::ArmMode_e::RIGHT;
            }
            if (IS_LAST_KEY_VT13(C) && !IS_KEY_VT13(C)) {
                armMsg.mode = ARM::ArmMode_e::TWO;
            }
        }
        //一键: 取矿*3，从矿仓取矿，公路区跨越, 夹爪
        if (IS_KEY_VT13(Q))
            armMsg.oneClick = ARM::OneClick_e::ORE_PICK1;
        if (IS_KEY_VT13(E))
            armMsg.oneClick = ARM::OneClick_e::ORE_PICK2;
        if (IS_KEY_VT13(R))
            armMsg.oneClick = ARM::OneClick_e::ORE_PICK3;
        if (IS_KEY_VT13(F))
            armMsg.oneClick = ARM::OneClick_e::HIGHWAY_CROSS;
        if (IS_KEY_VT13(G))
            armMsg.oneClick = ARM::OneClick_e::ORE_BIN_EXTRACT;
        if (IS_KEY_VT13(V) && !IS_LAST_KEY_VT13(V))
            armMsg.switch1 = !armMsg.switch1;
#endif
    }

    notify(&chassisMsg, msgBus_->chassisQueue);
    notify(&gimbalMsg, msgBus_->gimbalQueue);
    notify(&armMsg, msgBus_->armQueue);
}


#else
template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::masterHandle()
{
    ChassisMsg_s chassisMsg{};

    // User-defined processing logic goes here ↓
    if constexpr (DEV_TYPE == RCDevType_e::DT7) {
        using SW = DT7::Sw_e;
        DT7::RcData_s *rcData = static_cast<DT7::RcData_s *>(rc_->getData());

        updateRocker(rx, (float)rcData->rc.ch0);
        updateRocker(ry, (float)rcData->rc.ch1);
        updateRocker(lx, (float)rcData->rc.ch2);

        if (rcData->rc.switchRight == SW::DOWN) {
            chassisMsg.state = CHASSIS::FSMState_e::STOP;
            chassisMsg.vx = 0.f;
            chassisMsg.vy = 0.f;
            chassisMsg.yaw = 0.f;
        } else {
            if (rcData->rc.switchLeft == SW::DOWN) {
                chassisMsg.state = CHASSIS::FSMState_e::RUN;
            } else {
                chassisMsg.state = CHASSIS::FSMState_e::SPIN;
            }
            chassisMsg.vx = SMOOTH::sCurve(INPUT_VX_MAX, ry, T_ACC_CNT);
            chassisMsg.vy = -SMOOTH::sCurve(INPUT_VY_MAX, rx, T_ACC_CNT);
            chassisMsg.yaw = SMOOTH::sCurve(INPUT_YAW_MAX, lx, T_ACC_CNT);
        }

#if APP_USE_DT7_KEYBOARD
        float accK = 1.f, lxCommand = 0;
        chassisMsg.state = CHASSIS::FSMState_e::RUN;
        //加速
        if (IS_KEY_DT7(SHIFT))
            accK = 1.5f;
        else
            accK = 1.f;

        //前后左右
        if (IS_KEY_DT7(W)) {
            updateRocker(ry, 1); //加速向前，ry 增加并趋近于最大值。
        } else if (IS_KEY_DT7(S)) {
            updateRocker(ry, -1); //加速向后，ry 减少并趋近于最小值。
        } else {
            updateRocker(ry, 0); //自然减速回中，ry 向 0 值靠近。
        }

        if (IS_KEY_DT7(A)) {
            updateRocker(rx, -1);
        } else if (IS_KEY_DT7(D)) {
            updateRocker(rx, 1);
        } else {
            updateRocker(rx, 0);
        }

        //旋转
        if (rcData->mouse.pressRight == 1)
            lxCommand = 1;
        else if (rcData->mouse.pressLeft == 1)
            lxCommand = -1;
        else
            lxCommand = 0;

        updateRocker(lx, lxCommand);

        chassisMsg.vx = SMOOTH::sCurve(accK * Chassis::MAX_VX_SPEED, ry, T_ACC_CNT);  // Scale to m/s
        chassisMsg.vy = SMOOTH::sCurve(accK * Chassis::MAX_VY_SPEED, -rx, T_ACC_CNT); // Scale to m/s
        chassisMsg.yaw = SMOOTH::sCurve(accK * Chassis::MAX_WZ_SPEED, lx, T_ACC_CNT); // Scale to m/s
#endif
    } else if constexpr (DEV_TYPE == RCDevType_e::ET08A) {
        ET08A::RcData_s *rcData = static_cast<ET08A::RcData_s *>(rc_->getData());
        (void)rcData;
    } else if constexpr (DEV_TYPE == RCDevType_e::VT13) {
        VT13::RcData_s *rcData = static_cast<VT13::RcData_s *>(rc_->getData());
        (void)rcData;
    }
    notify(&chassisMsg, msgBus_->chassisQueue);
}
#endif

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}

template <RCDevType_e DEV_TYPE> void RcMsgHandler<DEV_TYPE>::updateRocker(float &_target, float _channel)
{
    float delta = (_channel * T_ACC_CNT / channelMax_) - _target;
    _target += std::fmax(-S_CURVE_ACC, std::fmin(S_CURVE_ACC, delta));
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
