#pragma once
#include "MsgBase.hpp"
#include "FreeRTOS.h"
#include "queue.h"

namespace CHASSIS {
enum class FSMState_e : uint8_t;
}

struct MsgBus_s {
    QueueHandle_t chassisQueue;
    QueueHandle_t gimbalQueue;
    QueueHandle_t armQueue;
};

struct chassisMsg : public Msg {
    float vx, vy, wz;
};

struct gimbalMsg : public Msg {
    float pitch, yaw;
};

struct armMsg : public Msg {
    float j1, j2, j3, j4, j5, j6;
};

struct ChassisTxMsg_s {
    CHASSIS::FSMState_e state;
};

struct GimbalTxMsg_s {};

struct ArmTxMsg_s {};

struct RefereeTxMsg_s {};
