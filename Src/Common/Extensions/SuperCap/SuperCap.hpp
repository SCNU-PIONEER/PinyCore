#pragma once
#include "./SuperCapMsg.hpp"

class SuperCap {
public:
    static constexpr float DEFAULT_CAP_TX_FREQ = 200.f;
    static constexpr float CAP_ENERGY_MAX = 255.f;
    static constexpr float OFFLINE_FREQ_THRESHOLD = 10.f;

    enum class State_e : uint8_t {
        OFFLINE = 0u,
        ONLINE = 1u,
    };

    SuperCap(canHandle *_hcan, uint16_t _cmdId = 0x222, uint16_t _dataId = 0x223, float _txFreq = DEFAULT_CAP_TX_FREQ);

    void task();

    void enable();
    void disable();
    void limitCharge();
    void unlimitCharge();
    void setChargelimitRatio(float _ratio); // [0, 1]
    void setClearErrorFlag(bool _flag);
    void setSystemRestartFlag(bool _flag);
    void setChargePowerLimit(uint16_t _chargePowerLimit = 120);
    void setChargeEnergySlack(uint16_t _chargeEnergySlack = 60);

    State_e getState() const { return state_; }

    CapData_s &getCapData() { return capData_; }
    float getRxFreq() { return aux_.rxFreq; }

private:
    CapData_s capData_{};
    CapCmd_s capCmd_{};

    CapAUX_s aux_{};

    State_e state_ = State_e::OFFLINE;

    void registerCapCallback();

    void praseCapData(const uint8_t *_rxbuf);

    void rxFreqCalc();

    void computeRawCapCmd(RawCapCmd_s &_rawCmd) const;

    uint8_t capDataSend();

    bool isOnline() const;
};
