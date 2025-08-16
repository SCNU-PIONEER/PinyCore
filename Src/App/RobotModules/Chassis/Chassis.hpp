#pragma once

#include "Locomotion.hpp"
#include "FSMState.hpp"
#include "RcMsg.hpp"
#include "StmLog.hpp"
#include "MsgImpl.hpp"
#include "sdkconfig.h"
#include "StmLogMsg.hpp"


namespace CHASSIS {
enum class FSMState_e : uint8_t {
    STOP = 1, //停止
    LAUNCH,   //缓启动
    RUN,      //没头也能跑
    FOLLOW,   //底盘跟随
    SPIN,     //小陀螺
};
} // namespace CHASSIS

class Chassis {
public:
    static constexpr float MAX_VX_SPEED = 2.f;
    static constexpr float MAX_VY_SPEED = 2.f;
    static constexpr float MAX_WZ_SPEED = 3.f;
    static constexpr float MOUSE_CAL = 200.f;

    Chassis(CHASSIS::Locomotion *_wheel);

    float deltaYaw; //底盘和云台正向的差角，为0即重合

    void update(void *_param);

    StateFactory stateFactory;

    LOG::Logger &log = LOG::Logger::instance();

    chassisMsg msg = {};
    gimbalMsg gmsg = {};

    CHASSIS::Locomotion *wheel;

    void convertCoordinate();

    RC::RcRawMsg_t rcData;
};
