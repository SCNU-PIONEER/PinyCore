#pragma once

#include "MiniPc.hpp"

#include <cstdint>
#include <memory>

namespace MINIPC {

enum class SentryDecision_e : uint8_t {
    IDLE = 0,
    SPIN,   // 小陀螺
    NAVI,   // 导航，无自瞄
    AIMING, // 巡检，自瞄
};

#pragma pack(push, 1)
typedef struct {
    uint8_t navHeader; // 0xA6
    SentryDecision_e decision;
    float vx;
    float vy;
    float wz;
    uint16_t crcSum;
} NavToEc_s; //Navigation to electric control

typedef struct {
    uint8_t navHeader; // 0x6A
    float pitch;
    float yaw;
    uint16_t gameTime;
    uint8_t gameProgress; // 游戏时间
    uint8_t navHPstate;   // 导航的血量标志位
    uint16_t crcSum;
} EcToNav_s; //Electric control to Navigation
#pragma pack(pop)


class Nav : public MiniPc {
public:
    static constexpr uint8_t NAV_UART_REQ_SOF = 0x6A;
    static constexpr uint8_t NAV_UART_CTRL_SOF = 0xA6;
    static constexpr uint16_t NAV_DATA_LEN = (sizeof(NavToEc_s));

    Nav();
    void init(const EventGroupHandle_t &_event, uint32_t _eventBit) override;
    void usbIdleCallback(uint8_t *_buf, uint16_t _len, const EventGroupHandle_t &_event, uint32_t _eventBit) override;
    void parseData() override;
    void sendData() override;

    void rxFreqCalc();
    float getRxFreq() { return rxFreq_; }
    NavToEc_s &navToEData() { return navToEcDatas_; };
    EcToNav_s &ecToNavData() { return ecToNavDatas_; };

private:
    NavToEc_s navToEcDatas_;
    EcToNav_s ecToNavDatas_;
    uint8_t rxBuffer_[NAV_DATA_LEN];
    float rxFreq_;
    uint16_t rxCnt_ = 0;
};
} // namespace MINIPC
inline std::unique_ptr<MINIPC::Nav> nav;
