#pragma once

#include "ArmKinematic.hpp"
#include "TpMsg.hpp"
#include "stm32h7xx_hal.h"
#include <cstdint>
#include <FreeRTOS.h>
#include "event_groups.h"


#define TP_READY_EVENT (1 << 2) // 示教器事件

namespace TP {

class TeachPendant {
private:
    TeachPendant();
    TpCtrl_t teachJoint_;
    UART_HandleTypeDef *uart_;
    VtTpFrame_t *tpRxData;
    EventGroupHandle_t event_;

public:
    TeachPendant(const TeachPendant &) = delete;
    TeachPendant &operator=(const TeachPendant &) = delete;

    TpCtrl_t &getData() { return teachJoint_; }
    static TeachPendant &instance()
    {
        static TeachPendant instance_;
        return instance_;
    }

    void init(UART_HandleTypeDef *_huart, EventGroupHandle_t _event);
    void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos);
    static void rawCallBackFromISR(UART_HandleTypeDef *_huart, uint16_t _Pos);
    void convert();

    const Joint7D homePose = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };
};

} // namespace TP
