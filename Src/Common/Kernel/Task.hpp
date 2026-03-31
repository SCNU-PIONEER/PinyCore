#pragma once

#include "FreeRTOS.h"
#include "task.h"

enum class TaskPriority_e : uint8_t {
    LOW1 = 1,
    LOW2,
    LOW3,
    LOW4,
    LOW5,
    LOW6,
    LOW7,
    LOW8,
    LOW9,
    LOW10,
    MID1,
    MID2,
    MID3,
    MID4,
    MID5,
    MID6,
    MID7,
    MID8,
    MID9,
    MID10,
    HIGH1,
    HIGH2,
    HIGH3,
    HIGH4,
    HIGH5,
    HIGH6,
    HIGH7,
    HIGH8,
    HIGH9,
    HIGH10,
};

// refer to https://bitbucket.org/fjrg76/crtp_threadx/src/master/crtp_threadX.cpp
template <class Derived, size_t N> class Task {
public:
    virtual ~Task()
    {
        if (htask_ != nullptr) {
            vTaskDelete(htask_);
            htask_ = nullptr;
        }
    }

    TaskHandle_t &getTaskHandler() { return htask_; }

private:
    // to avoid constructor as a regular template class in CRTP
    Task(const char *const _name, TaskPriority_e _priority)
            : htask_(xTaskCreateStatic([](void *_param) { static_cast<Derived *>(_param)->task(); }, _name, N, this,
                                       static_cast<uint8_t>(_priority), stask, &TCB)) {};

    friend Derived;

    StackType_t stask[N];
    StaticTask_t TCB;
    TaskHandle_t htask_;
};
