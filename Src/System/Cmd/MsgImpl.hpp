#pragma once
#include "MsgBase.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include <cstdint>


namespace CHASSIS {
enum class FSMState_e : uint8_t;
}

namespace GIMBAL {
enum class FSMState_e : uint8_t;
}

namespace ARMORBOOSTER {
enum class FSMState_e : uint8_t;
}

namespace ARMORLOADER {
enum class FSMState_e : uint8_t;
}

struct MsgBus_s {
    QueueHandle_t chassisQueue;
    QueueHandle_t gimbalQueue;
    QueueHandle_t armorBoosterQueue;
    QueueHandle_t armorLoaderQueue;
    QueueHandle_t refereeQueue;
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ ctrl msg ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct ChassisMsg_s : public Msg {
    CHASSIS::FSMState_e state;
    float vx, vy, wz;
};

struct GimbalMsg_s : public Msg {
    GIMBAL::FSMState_e state;
    float wRoll, wPitch, wYaw;
};

struct ArmorBoosterMsg_s : public Msg {
    ARMORBOOSTER::FSMState_e state;
};

struct ArmorLoaderMsg_s : public Msg {
    ARMORLOADER::FSMState_e state;
    bool fire;
};

struct RefereeMsg_s : public Msg {
    uint8_t robotId;
    uint16_t remainHeat, gameTime, currentHP, shooterHeatLimit, chassisPowerLimit, chassisPowerBuffer;
    float bulletSpeed, rxFreq;
    bool isPlay;
};


//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ other msg ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct INSEularMsg_s : public Msg {
    float roll, pitch, yaw;
};

struct GimbalYawMsg_s : public Msg {
    bool isReset;
    float followYaw;
    float chassisYaw; // only single core system need it
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ temp cache ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct ChassisUIMsg_s {
    CHASSIS::FSMState_e state;
};

struct GimbalUIMsg_s {
    GIMBAL::FSMState_e state;
};

struct ArmorBoosterUIMsg_s {
    ARMORBOOSTER::FSMState_e state;
};
