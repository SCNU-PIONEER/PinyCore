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
enum class OneClick_e : uint8_t;
enum class ArmMode_e : uint8_t;
enum class FSMState_e : uint8_t;
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
    ARM::ArmMode_e mode;
    ARM::FSMState_e state;
    float j1, j2, j3, j4, j5, j6, j7, j8, j9, j10;
    ARM::Joint10 target;
    bool switch1; // ture: clamp, false: release
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
