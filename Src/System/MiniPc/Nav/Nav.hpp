#pragma once
#include "FreeRTOS.h"
#include "event_groups.h"
#include <cstdint>
#include <memory>

namespace MINIPC {
enum class SentryPosture_e : uint8_t {
    IDLE = 0,
    ATTACK,  // 攻击，1
    DEFENSE, // 防御，2
    MOVE,    // 移动，3
}; //默认姿态为移动

#pragma pack(push, 1)
typedef struct {
    uint8_t navHeader; // 0xA6
    SentryPosture_e posture;
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
    uint8_t gameProgress; // 比赛进程
    uint32_t currentChassisEnergy;
    uint16_t navHPstate;    // 导航的血量标志位
    uint32_t outpostStatus; //前哨站
    uint32_t enemyBaseStatus;
    uint32_t enemyOutpostStatus;
    uint16_t crcSum;
} EcToNav_s; //Electric control to Navigation
#pragma pack(pop)


class Nav {
public:
    static constexpr uint8_t NAV_UART_REQ_SOF = 0x6A;
    static constexpr uint8_t NAV_UART_CTRL_SOF = 0xA6;
    static constexpr uint16_t NAV_DATA_LEN = (sizeof(NavToEc_s));

    Nav(const EventGroupHandle_t &_event, uint32_t _eventBit);

    void sendData();
    void rxFreqCalc();
    void parseData();
    float getRxFreq() { return rxFreq_; }
    NavToEc_s &navToEData() { return navToEcDatas_; };
    EcToNav_s &ecToNavData() { return ecToNavDatas_; };

protected:
    void usbIdleCallback(uint8_t *_buf, uint16_t _len, const EventGroupHandle_t &_event, uint32_t _eventBit);

private:
    NavToEc_s navToEcDatas_;
    EcToNav_s ecToNavDatas_;
    uint8_t rxBuffer_[NAV_DATA_LEN];
    float rxFreq_;
    uint16_t rxCnt_ = 0;
};

} // namespace MINIPC

inline std::unique_ptr<MINIPC::Nav> nav = nullptr;
