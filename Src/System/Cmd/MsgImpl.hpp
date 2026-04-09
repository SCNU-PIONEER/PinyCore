#pragma once
#include "MsgBase.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include <array>


namespace CHASSIS {
enum class FSMState_e : uint8_t { STOP = 1, RUN, SPIN };
}

namespace GIMBAL {
enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN };
}


namespace ARM {
//一键：从矿仓取矿，公路穿越，资源区取矿（一次取两个，针对不同位置有123）
enum class OneClick_e : uint8_t { ORE_BIN_EXTRACT = 0, HIGHWAY_CROSS, ORE_PICK1, ORE_PICK2, ORE_PICK3 };
enum class FSMState_e : uint8_t { STOP = 0, NORMAL, PLAN, TEACH };
enum class ClampState_e : int8_t { NONE = 0, CLAMP = 1, RELEASE = 2 };
enum class CustomCtrlState_e : uint8_t { OFFLINE = 0, ONLINE = 1 };
using Joint = std::array<float, 6>;
}

struct MsgBus_s {
    QueueHandle_t chassisQueue;
    QueueHandle_t gimbalQueue;
    QueueHandle_t armQueue;
    QueueHandle_t refereeQueue;
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ ctrl msg ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct ChassisMsg_s : public Msg {
    CHASSIS::FSMState_e state = CHASSIS::FSMState_e::STOP;
    float vx, vy, yaw;
};

struct GimbalMsg_s : public Msg {
    GIMBAL::FSMState_e state = GIMBAL::FSMState_e::STOP;
    float pitch, yaw;
};

struct ArmMsg_s : public Msg {
    ARM::OneClick_e oneClick;
    ARM::FSMState_e state = ARM::FSMState_e::STOP;
    ARM::Joint target;
    ARM::ClampState_e switch1 = ARM::ClampState_e::NONE;
    bool isStucked = false;
    ARM::CustomCtrlState_e customCtrlisOnline = ARM::CustomCtrlState_e::OFFLINE;
    float videoYaw = 0.f;   // 图传 yaw，相对值
    float videoPitch = 0.f; // 图传 pitch, 相对值
};

struct RefereeMsg_s : public Msg {
    uint8_t robotId;
    uint16_t remainHeat, gameTime, currentHP, shooterHeatLimit, chassisPowerLimit, chassisPowerBuffer;
    float bulletSpeed, rxFreq;
    bool isPlay;
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ other msg ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct GimbalEularMsg_s : public Msg {
    float roll, pitch, yaw;
};

struct DeltaYawMsg_s : public Msg {
    float deltaYaw;
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ temp cache ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct ChassisUIMsg_s {
    CHASSIS::FSMState_e state;
};

struct GimbalUIMsg_s {
    GIMBAL::FSMState_e state;
};

struct ArmUIMsg_s {
    ARM::FSMState_e state;
    ARM::ClampState_e clampState;
    ARM::CustomCtrlState_e customCtrlisOnline;
};
