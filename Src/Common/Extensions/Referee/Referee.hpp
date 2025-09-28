/**
 * @file Referee.hpp
 * @brief 裁判系统类定义
 *
 * @version Version 1.0.0
 * @date 2025/9/21
 *
 * @copyright SCNU-PIONEER (c) 2025-2026
 *
 */
#pragma once

#include "RefereeProt.hpp"
#include "FreeRTOS.h"
#include <cstdint>
#include "event_groups.h"
#include "Bsp_uart.hpp"

#define REFEREE_READY_EVENT (1 << 2)

namespace REFEREE {

static constexpr uint8_t LEN_HEADER = 5;
static constexpr uint8_t LEN_CMDID = 2;
static constexpr uint8_t LEN_TAIL = 2;
static constexpr uint8_t SOF = 0xA5;

class RefReceiver {
    static constexpr uint16_t REFEREE_RX_BUFFER_LEN = 256;
    static constexpr uint8_t REFEREE_SYS_MAX_LOST = 5;

public:
    void init(UART_HandleTypeDef *_huart, EventGroupHandle_t _event);

    void uartIdleCallback(UART_HandleTypeDef *_huart);
    void readRefereeData();

    void incRxLost();
    bool isOffline();

    RefereeProt_s &getRefereeData() { return refereeData_; }

    static RefReceiver &instance()
    {
        static RefReceiver instance;
        return instance;
    }

private:
    RefReceiver();

    UART_HandleTypeDef *uart_;
    uint8_t *rxBuffer_;
    RefereeProt_s refereeData_;

    uint16_t rxLostCnt_ = 0;
    uint16_t lastPos = 0;
    uint16_t dataLen = 0;

    EventGroupHandle_t event_;
};

class RefereeTransmitter {
    static constexpr uint8_t REFEREE_TX_BUFFER_LEN = 128;

public:
    RefereeTransmitter(UART_HandleTypeDef *_huart);

    uint16_t sendData(uint16_t _cmdId, uint8_t *_pStruct, uint16_t _len);

private:
    UART_HandleTypeDef *uart_;

    uint8_t txBuffer_[REFEREE_TX_BUFFER_LEN];
};

} // namespace REFEREE
