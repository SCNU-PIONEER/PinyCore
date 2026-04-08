#pragma once
#include "Projdefs.hpp"
#include "Bsp.hpp"

#pragma pack(push, 1)
struct RawCapData_s {           // 0x052 (useNewFeedbackMessage = 1)
    uint8_t statusCode;         // 状态信息
    uint16_t chassisPower;      // 底盘功率，功率*64+16384 (-256W~+768W, 精度0.015625)
    uint16_t refereePower;      // 裁判系统功率，功率*64+16384 (-256W~+768W, 精度0.015625)
    uint16_t chassisPowerLimit; // 底盘最大可用功率（包括裁判系统）
    uint8_t capEnergy;          // 电容现有能量，0-255
};

struct CapCmd_s {
    uint8_t enableDCDC : 1;    // 允许启动DCDC
    uint8_t systemRestart : 1; // 系统重启
    uint8_t reserved1 : 3;
    uint8_t clearError : 1;    // 手动清除可清除的错误
    uint8_t enChargeLimit : 1; // 是否启用主动充电限制
    uint8_t useFeedback : 1;   // 是否使用反馈消息

    uint16_t powerLimit;      // 裁判限制功率，单位W
    uint16_t energyBuffer;    // 裁判能量缓冲，单位J
    uint8_t chargeRatioLimit; // 主动充电限制比例（能量），0-255
    int16_t reserved2;
};

struct CapState_s {
    uint8_t errorCode : 2;
    uint8_t limitFactor : 2;
    uint8_t wptStatus : 2;
    uint8_t useFeedback : 1;
    uint8_t capEnable : 1;
};
#pragma pack(pop)

struct CapData_s {
    CapState_s capState;     // 状态信息
    float chassisPower;      // 底盘功率，功率*64+16384 (-256W~+768W, 精度0.015625)
    float refereePower;      // 裁判系统功率，功率*64+16384 (-256W~+768W, 精度0.015625)
    float chassisPowerLimit; // 底盘最大可用功率（包括裁判系统）
    float capEnergyRatio;    // 电容现有能量，0-255
};

class CAP {
public:
    static constexpr float CAP_TX_FREQ = 200.f;
    static constexpr float DATA_RX_FREQ = 1000.f; //功率板中设置
    static constexpr float CAP_ENERGY_MAX = 255.f;

    CAP(canHandle *_hcan, uint16_t _cmdId = 0x222, uint16_t _dataId = 0x223);

    void registerCapCallback();
    void praseCapData(const uint8_t *_rxbuf);
    bool checkSend();
    void capTask(bool _capEnable, bool _systemRestart, bool _clearError, bool _enChargeLimit, uint8_t _chargeRatioLimit,
                 uint16_t _powerLimit);
    void rxFreqCalc();

    uint8_t capDataSend(bool _capEnable, bool _systemRestart, bool _clearError, bool _enChargeLimit,
                        uint8_t _chargeRatioLimit, uint16_t _powerLimit);

    CapData_s &getCapData() { return capData_; }
    float getRxFreq() { return rxFreq_; }

private:
    canHandle *hcan_;
    uint16_t cmdId_;
    uint16_t dataId_;

    CapData_s capData_;
    RawCapData_s rawCapData_;
    CapCmd_s capCmd_;

    uint32_t lastSendTick_ = 0.f;
    uint16_t rxCnt_ = 0;
    float rxFreq_ = 0.f;

    QueueHandle_t rxQueue_;
    PINYMOTOR::RxBus_s::CANRxBuf_s<8> rxBuf_ = {};
};
