#pragma once
#include "MsgBase.hpp"
#include "FreeRTOS.h"
#include "queue.h"


namespace CHASSIS {
enum class FSMState_e : uint8_t;
}

namespace GIMBAL {
enum class FSMState_e : uint8_t;
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
    float j1, j2, j3, j4, j5, j6;
};

struct RefereeMsg_s : public Msg {
    uint16_t currentHP, shooterHeatLimit, chassisPowerLimit, chassisPowerBuffer;
    float bulletSpeed;
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ other msg ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct GimbalEularMsg_s : public Msg {
    float roll, pitch, yaw;
};

struct DeltaYawMsg_s : public Msg {
    float deltaYaw;
};

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ temp cache ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
struct ChassisTxMsg_s {
    CHASSIS::FSMState_e state;
};

struct GimbalTxMsg_s {};

struct ArmTxMsg_s {};

struct RefereeTxMsg_s {};
