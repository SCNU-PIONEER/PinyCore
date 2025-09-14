#pragma once

#include "FSMState.hpp"
#include "TopicRouter.hpp"

namespace GIMBAL {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN, PATROL };
}

class Gimbal {
public:
    Gimbal()
            : insSub_(new Subscriber<INS_SYS::INSData_s>(
                      4, &TopicRouter::instance().insTopic, &insData_))
            , gimbalYawPub_(new Publisher<GimbalYawMsg_s>(
                      &TopicRouter::instance().gimbalYawTopic, &gimbalYawMsg))
    {
    }

    const INS_SYS::INSData_s &ins() const { return insData_; }

    float ctrlYawAng() const { return ctrlYawAng_; }

    float baseYawAng() const { return baseYawAng_; }

    GimbalMsg_s msg = {};

    GimbalYawMsg_s gimbalYawMsg{};

protected:
    StateFactory<GIMBAL::FSMState_e> stateFactory_;

    INS_SYS::INSData_s insData_ = {};
    Subscriber<INS_SYS::INSData_s> *insSub_;

    // Master send deltaYaw to chassis
    Publisher<GimbalYawMsg_s> *gimbalYawPub_;

    float ctrlYawAng_ = 0.f;        // end's ang in earth,主控yaw的角度
    float constDeltaYawArc_ = 0.f;  // 上电后就有的底盘和云台系的差角
    bool isConfirmYawZero_ = false; // 确认是否两系转换完成

    float baseYawAng_ = 0.f; // 底盘跟随，单板要用，双板直接读取imu
};
