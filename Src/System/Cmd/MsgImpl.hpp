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

namespace ARMORBOOSTER {
enum class FSMState_e : uint8_t;
}
namespace ARM {
//一键：从矿仓取矿，公路穿越，资源区取矿（一次取两个，针对不同位置有123）
enum class OneClick_e : uint8_t { ORE_BIN_EXTRACT = 0, HIGHWAY_CROSS, ORE_PICK1, ORE_PICK2, ORE_PICK3 };
enum class ArmMode_e : uint8_t { LEFT = 0, RIGHT, TWO };
enum class FSMState_e : uint8_t { STOP = 0, NORMAL, PLAN, TEACH };
using Joint10 = std::array<float, 10>;
}

struct MsgBus_s {
    QueueHandle_t chassisQueue;
    QueueHandle_t gimbalQueue;
    QueueHandle_t armQueue;
    QueueHandle_t refereeQueue;
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ ctrl msg ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct ChassisMsg_s : public Msg {
    CHASSIS::FSMState_e state;
    float vx, vy, yaw;
};

struct GimbalMsg_s : public Msg {
    GIMBAL::FSMState_e state;
    float roll, pitch, yaw;
};

struct ArmMsg_s : public Msg {
    ARM::OneClick_e oneClick;
    ARM::ArmMode_e mode = ARM::ArmMode_e::RIGHT;
    ARM::FSMState_e state;
    ARM::Joint10 target;
    bool switch1 = false; // ture: clamp, false: release
    bool isStucked = false;
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
};

struct ArmorBoosterUIMsg_s {
    ARMORBOOSTER::FSMState_e state;
};
