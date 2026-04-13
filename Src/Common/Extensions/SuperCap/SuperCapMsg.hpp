#pragma once

#include <cstdint>
#include <cstring>

#include "Bsp.hpp"

#pragma pack(push, 1)

struct RawCapCmd_s {
    uint8_t enableDCDC : 1;
    uint8_t systemRestart : 1;
    uint8_t reserved1 : 3;
    uint8_t clearError : 1;
    uint8_t enChargeLimit : 1;
    uint8_t useFeedback : 1;

    uint16_t chargePowerLimit;
    uint16_t chargeEnergySlack;
    uint8_t chargeRatioLimit;
    int16_t reserved2;
};

struct CapState_s {
    uint8_t errorCode : 2;
    uint8_t limitFactor : 2;
    uint8_t wptStatus : 2;
    uint8_t useFeedback : 1;
    uint8_t capEnable : 1;
};

struct RawCapData_s {
    uint8_t statusCode;
    uint16_t outputPower;
    uint16_t inputPower;
    uint16_t outputPowerMax;
    uint8_t capEnergy;

    RawCapData_s(const uint8_t *_buf) { memcpy(this, _buf, sizeof(RawCapData_s)); }
};

struct CapData_s {
    CapState_s capState;  // 状态信息
    float outputPower;    // 电容控制板输出功率，原始值*64+16384 (-256W~+768W, 精度0.015625)
    float inputPower;     // 电容控制板输入功率，原始值*64+16384 (-256W~+768W, 精度0.015625)
    float outputPowerMax; // 估计的最大输出功率
    float capEnergyRatio; // 电容现有能量，[0,1]
};

#pragma pack(pop)

struct CapCmd_s {
    bool capEnable;             // 允许启动DCDC
    bool systemRestart;         // 系统重启
    bool clearError;            // 手动清除可清除的错误
    bool enChargeLimit;         // 是否启用主动充电限制
    float chargeRatioLimit;     // 主动充电限制比例（能量），[0,1]
    uint16_t chargePowerLimit;  // 电容最大取电(充电)功率，单位W
    uint16_t chargeEnergySlack; // 取电松弛能量，单位J
};

struct CapAUX_s {
    canHandle *hcan;
    uint16_t cmdId;

    uint32_t lastSendTick;
    uint32_t txPeriodTicks;
    uint16_t rxCnt;
    float rxFreq;

    QueueHandle_t rxQueue;
    uint8_t rxBuf[8];
};