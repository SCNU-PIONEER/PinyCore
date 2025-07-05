#pragma once
#include "MsgBase.hpp"
#include "FreeRTOS.h"
#include "queue.h"

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
