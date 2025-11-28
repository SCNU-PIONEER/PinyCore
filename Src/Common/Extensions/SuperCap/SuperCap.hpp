#pragma once
#include "Projdefs.hpp"
#include "Bsp.hpp"

static constexpr float CAP_VOLTAGE_MAX = 26.f; // 电容最大电压
static constexpr float CAP_VOLTAGE_MIN = 5.f;  // 电容最小
static constexpr float CAP_CURRENT_MAX = 15.f; // 电容最大电流

#pragma pack(push, 1)
struct RawCapData_s {
    uint8_t setPower : 8;       // 主控设置的输入功率
    uint16_t inputCurrent : 13; // 功率板采集的母线输入电流
    int16_t chargeCurrent : 14; // 功率板采集的超电充电电流
    int16_t busVoltage : 11;    // 功率板认为的母线电压
    uint16_t capVoltage : 11;   // 功率板认为的电容电压
    uint8_t remainEnergery : 5; // 功率板认为的超电剩余存储能量

    uint8_t CapEnableFlag : 1;
    uint8_t LowVoltageFlag : 1;
};

struct CapCmd_s {
    float chargePower; // 4bytes
    uint8_t EnableCAP; // 1bytes
    uint8_t EnableCharge;
    uint16_t chassisCmdPower;
};
#pragma pack(pop)
struct CapData_s {
    uint8_t CapEnableFlag;
    uint8_t CapEnableCharge;
    uint8_t LowVoltageFlag;

    float inputCurrent;  // (电池端)输入电流,单位:A
    float outputCurrent; // (电机端)输出电流,单位:A
    float inputVoltage;  // (电池端)输入电压,单位:V
    float capVoltage;    // (电容端)输出电压,单位:V
    float powerSet;      // 功率板内部功率限制设定值,单位:W
};

class CAP {
public:
    static constexpr uint16_t CAP_CMD_ID = 0x222;
    static constexpr uint16_t CAP_DATA_ID = 0x223;
    static constexpr float CAP_TX_FREQ = 50.f;
    static constexpr float DATA_RX_FREQ = 1000.f;  //  应与底盘控制频率相同
    static constexpr float BATTERY_VOLTAGE = 24.f; // 电池电压

    CAP(canHandle *_hcan);

    void registerCapCallback();
    void praseCapData(const uint8_t *_rxbuf);
    bool checkSend();
    void capTask(float _capChargePower, bool _capEnableFlag, bool _enableCharge,
                 uint16_t _chassisPower);
    void rxFreqCalc();

    uint8_t capDataSend(float _capChargePower, bool _capEnableFlag,
                        bool _enableFeedforward, uint16_t _chassisPower);

    CapData_s &getCapData() { return capData_; }
    float getRxFreq() { return rxFreq_; }

    float chargeCmdPower = 0.f; //期望电容充电功率

private:
    canHandle *hcan_;

    CapData_s capData_;
    RawCapData_s rawCapData_;
    CapCmd_s capCmd_;

    uint32_t lastSendTick_ = 0.f;
    uint16_t rxCnt_ = 0;
    float rxFreq_ = 0.f;

    QueueHandle_t rxQueue_;
    PINYMOTOR::RxBus_s::CANRxBuf_s<8> rxBuf_ = {};
};
