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

template <> void RcMsgHandler<RCDevType_e::DT7>::masterHandle()
{
    static ChassisMsg_s chassisMsg{};
    static GimbalMsg_s gimbalMsg{};
    static ArmMsg_s armMsg{};

    using SW = DT7::Sw_e;
    DT7::RcData_s *rcData = static_cast<DT7::RcData_s *>(rc_->getData());

    updateRocker(rx, rcData->rc.ch0);
    updateRocker(ry, rcData->rc.ch1);
    updateRocker(lx, rcData->rc.ch2);
    updateRocker(ly, rcData->rc.ch3);

    // rx = ((float)rcData->rc.ch0 * 100 / 660);
    // ry = ((float)rcData->rc.ch1 * 100 / 660);
    // lx = ((float)rcData->rc.ch2 * 100 / 660);
    // ly = ((float)rcData->rc.ch3 * 100 / 660);

    gimbalVal = -((float)rcData->rc.ch2 * GIMBAL_YAW_MAX / 660);
    //拨轮控夹爪
    // if (rcData->wheel == 1684)
    //     armMsg.switch1 = ARM::ClampState_e::CLAMP;
    // else if (rcData->wheel > 7000)
    // armMsg.switch1 = ARM::ClampState_e::RELEASE;
    if (rcData->rc.switchRight == SW::DOWN) {
        //arm
        armMsg.switch1 = ARM::ClampState_e::NONE;
        armMsg.state = ARM::FSMState_e::STOP;
        armMsg.switch1 = ARM::ClampState_e::NONE;
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
            // if (rcData->rc.ch1 == 660) { //ry
            //     chassisMsg.state = CHASSIS::FSMState_e::SPIN;
            // } else if (rcData->rc.ch1 == -660) { //ry
            //     // chassis
            //     chassisMsg.state = CHASSIS::FSMState_e::RUN;
            // }
            chassisMsg.state = CHASSIS::FSMState_e::RUN;
            chassisMsg.vx = ry / 100 * INPUT_VX_MAX;
            chassisMsg.vy = -rx / 100 * INPUT_VY_MAX;
        } else if (rcData->rc.switchLeft == SW::MID) {
            //arm
            armMsg.state = ARM::FSMState_e::NORMAL;
            armMsg.target[0] = -lx / 314 / 50; //8016yaw
            armMsg.target[1] = ly / 314 / 50;  //10010pitch
            armMsg.target[2] = ry / 314 / 50;  //8009pitch
            armMsg.target[3] = rx / 314 / 50;  //4340yaw

        } else if (rcData->rc.switchLeft == SW::UP) {
            //arm
            armMsg.state = ARM::FSMState_e::NORMAL;
            armMsg.target[4] = -lx / 314 / 50; //3507roll
            armMsg.target[5] = -ly / 314 / 50; //3507pitch

            //夹爪
            if (rcData->rc.ch1 == 660) //ry up
                armMsg.switch1 = ARM::ClampState_e::RELEASE;
            else if (rcData->rc.ch1 == -660) //ry down
                armMsg.switch1 = ARM::ClampState_e::CLAMP;
        }
    } else if (rcData->rc.switchRight == SW::UP) {
        armMsg.videoYaw = (float)rcData->rc.ch0 / 660;
        armMsg.videoPitch = (float)rcData->rc.ch1 / 660;

        if (rcData->rc.switchLeft == SW::UP) {
            if (rcData->rc.ch0 == 660 && rcData->rc.ch1 == 660) { // 右上
                __set_FAULTMASK(1);
                NVIC_SystemReset();
            }
        }

        // if (rcData->rc.ch1 == 660) {
        //     if (((VT13 *)rc_.get())->isOnline()) {
        //         armMsg.state = ARM::FSMState_e::TEACH;
        //         auto customData = ((VT13 *)rc_.get())->getCtrlData();
        //         armMsg.target[0] = customData.joint[0]; //8016yaw
        //         armMsg.target[1] = customData.joint[1]; //10010pitch
        //         armMsg.target[2] = customData.joint[2]; //8009pitch
        //         armMsg.target[3] = customData.joint[3]; //4340yaw
        //         armMsg.target[4] = customData.joint[4]; //3507pitch
        //         armMsg.target[5] = customData.joint[5]; //3507roll
        //     }
        // }
        // if (armMsg.state == ARM::FSMState_e::TEACH) {
        //     auto customData = ((VT13 *)rc_.get())->getCtrlData();
        //     armMsg.target[0] = customData.joint[0]; //8016yaw
        //     armMsg.target[1] = customData.joint[1]; //10010pitch
        //     armMsg.target[2] = customData.joint[2]; //8009pitch
        //     armMsg.target[3] = customData.joint[3]; //4340yaw
        //     armMsg.target[4] = customData.joint[4]; //3507pitch
        //     armMsg.target[5] = customData.joint[5]; //3507roll
        // }
    }
}

template <> void RcMsgHandler<RCDevType_e::ET08A>::masterHandle()
{
    ET08A::RcData_s *rcData = static_cast<ET08A::RcData_s *>(rc_->getData());
    (void)rcData;
};

template <> void RcMsgHandler<RCDevType_e::VT13>::masterHandle()
{
    VT13::RcData_s *rcData = static_cast<VT13::RcData_s *>(rc_->getData());
#if APP_USE_VT13_KEYBOARD
    /*
         * gimbal:STOP,RUN,
         * chassis:STOP,RUN,SPIN
         * arm:ONECLICK,MODE,STOP,NORMAL,PLAN,TEACH,夹爪
         */
    //机械臂控制使用自定义控制器
    /*————————————————————————————————————————————————chassis——————————————————————————————————————————————*/

    if (rcData->gamepad.sw == VT13::Sw_e::S) {
        chassisMsg.state = CHASSIS::FSMState_e::STOP;
        gimbalMsg.state = GIMBAL::FSMState_e::STOP;
        armMsg.state = ARM::FSMState_e::STOP;
    } else if (rcData->gamepad.sw == VT13::Sw_e::N) {
        chassisMsg.state = CHASSIS::FSMState_e::RUN;
        gimbalMsg.state = GIMBAL::FSMState_e::RUN;
        armMsg.state = ARM::FSMState_e::NORMAL;
    }

    if (keyBoardCtrl) {
        if (IS_KEY_VT13(CTRL) && IS_KEY_VT13(Z)) {
            chassisMsg.state = CHASSIS::FSMState_e::RUN;
            gimbalMsg.state = GIMBAL::FSMState_e::RUN;
            armMsg.state = ARM::FSMState_e::NORMAL;


        } else if (IS_KEY_VT13(CTRL) && IS_KEY_VT13(C)) {
            chassisMsg.state = CHASSIS::FSMState_e::STOP;
            gimbalMsg.state = GIMBAL::FSMState_e::STOP;
            armMsg.state = ARM::FSMState_e::STOP;
        }


        float accK = 1.f;
        if (chassisMsg.state == CHASSIS::FSMState_e::RUN) {
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

            chassisMsg.vx = SMOOTH::sCurve(accK * Chassis::MAX_VX_SPEED, ry, T_ACC_CNT);  // Scale to m/s
            chassisMsg.vy = SMOOTH::sCurve(accK * Chassis::MAX_VY_SPEED, -rx, T_ACC_CNT); // Scale to m/s
        } else {
            chassisMsg.vx = 0.f;
            chassisMsg.vy = 0.f;
        }


        // if (IS_KEY_VT13(SHIFT)) {
        //     // 云台图传, 底盘不动
        //     armMsg.videoYaw = -(rcData->KBM.mouseX);
        //     armMsg.videoPitch = -(rcData->KBM.mouseY);
        // } else {
        //     // 云台旋转，底盘自动跟随
        //     gimbalMsg.yaw = -(float)(rcData->KBM.mouseX) * GIMBAL_YAW_MAX;
        // }

        // 云台旋转，底盘自动跟随
        gimbalMsg.yaw = -(float)(rcData->KBM.mouseX) * GIMBAL_YAW_MAX;


        /*——————————————————————————————————————————————————————————————————————————————————————————————————————————*/
        //单双臂, 鼠标右键切换，双臂模式只有自定义或者一键模式可用
        // if (rcData->KBM.mouseRight) {
        //     if (armMsg.mode == ARM::ArmMode_e::LEFT)
        //         armMsg.mode = ARM::ArmMode_e::RIGHT;
        //     else if (armMsg.mode == ARM::ArmMode_e::RIGHT)
        //         armMsg.mode = ARM::ArmMode_e::LEFT;
        // }
        if (IS_KEY_VT13(R)) {
            armMsg.state = ARM::FSMState_e::TEACH;
            auto customData = ((VT13 *)rc_.get())->getCtrlData();
            armMsg.target[0] = customData.joint[0]; //8016yaw
            armMsg.target[1] = customData.joint[1]; //10010pitch
            armMsg.target[2] = customData.joint[2]; //8009pitch
            armMsg.target[3] = customData.joint[3]; //4340yaw
            armMsg.target[4] = customData.joint[4]; //3507pitch
            armMsg.target[5] = customData.joint[5]; //3507roll

            //一键: 取矿*3，从矿仓取矿，公路区跨越, 夹爪
            // if (IS_KEY_VT13(CTRL) && IS_KEY_VT13(Q))
            //     armMsg.oneClick = ARM::OneClick_e::ORE_PICK1;
            // if (IS_KEY_VT13(CTRL) && IS_KEY_VT13(E))
            //     armMsg.oneClick = ARM::OneClick_e::ORE_PICK2;
            // if (IS_KEY_VT13(CTRL) && IS_KEY_VT13(R))
            //     armMsg.oneClick = ARM::OneClick_e::ORE_PICK3;
            // if (IS_KEY_VT13(CTRL) && IS_KEY_VT13(F))
            //     armMsg.oneClick = ARM::OneClick_e::HIGHWAY_CROSS;
            // if (IS_KEY_VT13(CTRL) && IS_KEY_VT13(G))
            //     armMsg.oneClick = ARM::OneClick_e::ORE_BIN_EXTRACT;
            // if (IS_KEY_VT13(V) && !IS_LAST_KEY_VT13(V)) {
            //     if (armMsg.switch1 == ARM::ClampState_e::CLAMP)
            //         armMsg.switch1 = ARM::ClampState_e::RELEASE;
            //     else if (armMsg.switch1 == ARM::ClampState_e::RELEASE)
            //         armMsg.switch1 = ARM::ClampState_e::CLAMP;
            // }
        }

        notify(&chassisMsg, msgBus_->chassisQueue);
        notify(&gimbalMsg, msgBus_->gimbalQueue);
        notify(&armMsg, msgBus_->armQueue);
    }
#endif

    updateRocker(rx, rcData->gamepad.ch[0]);
    updateRocker(ry, rcData->gamepad.ch[1]);
    updateRocker(lx, rcData->gamepad.ch[2]);
    updateRocker(ly, rcData->gamepad.ch[3]);

    // if (rcData->gamepad.wheel == -660)
    //     armMsg.switch1 = ARM::ClampState_e::CLAMP;
    // else if (rcData->gamepad.wheel > 660)
    //     armMsg.switch1 = ARM::ClampState_e::RELEASE;
    if (rcData->gamepad.sw == VT13::Sw_e::S) {
        //arm
        armMsg.switch1 = ARM::ClampState_e::NONE;
        armMsg.state = ARM::FSMState_e::STOP;
        armMsg.switch1 = ARM::ClampState_e::NONE;
        // gimbal
        gimbalMsg.state = GIMBAL::FSMState_e::STOP;
        gimbalMsg.yaw = 0.f;
        // chassis
        chassisMsg.state = CHASSIS::FSMState_e::STOP;
        chassisMsg.vx = chassisMsg.vy = 0.f;
    } else if (rcData->gamepad.sw == VT13::Sw_e::N) {
        //拨轮控云台
        gimbalMsg.state = GIMBAL::FSMState_e::RUN;
        gimbalMsg.yaw = -((float)rcData->gamepad.wheel * GIMBAL_YAW_MAX / 660);

        if (rcData->gamepad.trigger) {
            chassisMsg.state = CHASSIS::FSMState_e::SPIN;
        }
        chassisMsg.vx = ry / 100 * INPUT_VX_MAX;
        chassisMsg.vy = -rx / 100 * INPUT_VY_MAX;

        armMsg.state = ARM::FSMState_e::NORMAL;
        armMsg.target[0] = -lx / 314 / 50; //8016yaw
        armMsg.target[1] = ly / 314 / 50;  //10010pitch
    } else if (rcData->gamepad.sw == VT13::Sw_e::C) {
        //arm
        armMsg.state = ARM::FSMState_e::NORMAL;
        armMsg.target[2] = ry / 314 / 50;  //8009pitch
        armMsg.target[3] = rx / 314 / 50;  //4340yaw
        armMsg.target[4] = -lx / 314 / 50; //3507roll
        armMsg.target[5] = -ly / 314 / 50; //3507pitch
    }

    // if (rcData->gamepad.ch[1] == 660) {
    //     if (((VT13 *)rc_.get())->isOnline()) {
    //         armMsg.state = ARM::FSMState_e::TEACH;
    //         auto customData = ((VT13 *)rc_.get())->getCtrlData();
    //         armMsg.target[0] = customData.joint[0]; //8016yaw
    //         armMsg.target[1] = customData.joint[1]; //10010pitch
    //         armMsg.target[2] = customData.joint[2]; //8009pitch
    //         armMsg.target[3] = customData.joint[3]; //4340yaw
    //         armMsg.target[4] = customData.joint[4]; //3507pitch
    //         armMsg.target[5] = customData.joint[5]; //3507roll
    //     }
    // }
    // if (armMsg.state == ARM::FSMState_e::TEACH) {
    //     auto customData = ((VT13 *)rc_.get())->getCtrlData();
    //     armMsg.target[0] = customData.joint[0]; //8016yaw
    //     armMsg.target[1] = customData.joint[1]; //10010pitch
    //     armMsg.target[2] = customData.joint[2]; //8009pitch
    //     armMsg.target[3] = customData.joint[3]; //4340yaw
    //     armMsg.target[4] = customData.joint[4]; //3507pitch
    //     armMsg.target[5] = customData.joint[5]; //3507roll
    // }

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
