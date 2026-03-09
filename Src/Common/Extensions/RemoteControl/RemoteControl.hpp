#pragma once

#include "Bsp.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"
#include <atomic>

class RemoteControl {
public:
    RemoteControl(UART_HandleTypeDef *_huart, uint16_t _bufLen, EventGroupHandle_t &_event)
            : uart_(_huart), buf_((uint8_t *)Dma::instance().ram_alloc(_bufLen)), event_(_event) {};
    virtual ~RemoteControl() = default;
    virtual void parse() = 0;
    virtual bool isOnline() = 0;
    virtual void *getData() = 0;

protected:
    virtual void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos, uint32_t _eventBit) = 0;

    Uart uart_;
    uint8_t *buf_ = nullptr;
    EventGroupHandle_t &event_;
    std::atomic<uint8_t> rxLostCnt_;
};
