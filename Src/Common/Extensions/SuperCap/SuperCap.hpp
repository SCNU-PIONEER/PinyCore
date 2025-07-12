#pragma once
#include <cstdint>
#include "FreeRTOS.h"
#include "task.h"

#define VCapMAX     26.f
#define VCapMIN     5.f
#define CAPCurrMax  15.f
#define CAPCmdID    0x222
#define CAPMasterID 0x223


struct __attribute__((packed)) RawCapData_s {
    uint8_t setPower : 8;       // 主控设置的输入功率
    uint16_t inputCurrent : 13; // 功率板采集的母线输入电流
    int16_t chargeCurrent : 14; // 功率板采集的超电充电电流
    int16_t busVoltage : 11;    // 功率板认为的母线电压
    uint16_t capVoltage : 11;   // 功率板认为的电容电压
    uint8_t remainEnergery : 5; // 功率板认为的超电剩余存储能量
    bool CapEnableFlag;
    bool LowVoltageFlag;
};

struct CapData_s {
    uint8_t CapEnableFlag;
    uint8_t CapEnableFeedforward;
    uint8_t LowVoltageFlag;

    float inputCurrent;  // (电池端)输入电流,单位:A
    float outputCurrent; // (电机端)输出电流,单位:A
    float inputVoltage;  // (电池端)输入电压,单位:V
    float capVoltage;    // (电容端)输出电压,单位:V
    float powerSet;      // 功率板内部功率限制设定值,单位:W
};

struct __attribute__((packed)) CapCmd_s {
    float chargeCmdPower; // 4bytes
    uint8_t EnableCAP;    // 1bytes
    uint8_t EnableFeedforward;
    uint16_t chassisCmdPower;
};

class CAP {
public:
    CAP();

    void registerCapCallback();

    void praseCapData(const uint8_t *_rxbuf);

    bool checkSend();

    uint8_t capDataSend(float _capChargePower, bool _capEnableFlag,
                        bool _EnableFeedforward, bool _limitPower);

    CapData_s &getCapData() { return capData_; }

private:
    CapData_s capData_;
    RawCapData_s rawCapData_;
    CapCmd_s capCmd_;

    uint32_t lastSendTick_ = 0.f;

    uint32_t capTxFreq_;
};
