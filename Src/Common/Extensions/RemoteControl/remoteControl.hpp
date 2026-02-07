#pragma once

#include <memory>
#include "Bsp.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"

class RemoteControl {
public:
    static constexpr uint16_t W = 0x01 << 0;
    static constexpr uint16_t S = 0x01 << 1;
    static constexpr uint16_t A = 0x01 << 2;
    static constexpr uint16_t D = 0x01 << 3;
    static constexpr uint16_t SHIFT = 0x01 << 4;
    static constexpr uint16_t CTRL = 0x01 << 5;
    static constexpr uint16_t Q = 0x01 << 6;
    static constexpr uint16_t E = 0x01 << 7;
    static constexpr uint16_t R = 0x01 << 8;
    static constexpr uint16_t F = 0x01 << 9;
    static constexpr uint16_t G = 0x01 << 10;
    static constexpr uint16_t Z = 0x01 << 11;
    static constexpr uint16_t X = 0x01 << 12;
    static constexpr uint16_t C = 0x01 << 13;
    static constexpr uint16_t V = 0x01 << 14;
    static constexpr uint16_t B = 0x01 << 15;

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
    volatile uint8_t rxLostCnt_;
};

inline std::unique_ptr<RemoteControl> rc = nullptr;
