#pragma once

#include <cstdint>

#include "MsgImpl.hpp"
#include "sdkconfig.h"
#include "BSP.hpp"
#include "event_groups.h"
#include GIMBAL_FILE
// #include "Vision.hpp"


#define NAV_READY_EVENT (1 << 3)

namespace NAV {

enum class SentryDecision_e : uint8_t {
    IDLE = 0,
    SPIN,   // 小陀螺
    NAVI,   // 导航，无自瞄
    AIMING, // 巡检，自瞄
};

#pragma pack(push, 1)
typedef struct {
    uint8_t header; // 0xA5
    SentryDecision_e decision;
    float vx;
    float vy;
    float wz;
    uint16_t crcSum;
} NavToEc_s; //Navigation to electric control

typedef struct {
    uint8_t header; // 0x5A
    float pitch;
    float yaw;
    uint16_t gameTime;
    uint8_t gameProgress; // 游戏时间
    uint8_t navHPstate;   // 导航的血量标志位
    uint16_t crcSum;
} EcToNav_s; //Electric control to Navigation
#pragma pack(pop)

class Nav {
public:
    static constexpr uint8_t NAV_RX_LOST_MAX = 100; // 导航丢失计数
    static constexpr uint8_t NAV_UART_REQ_SOF = 0x5A;
    static constexpr uint8_t NAV_UART_CTRL_SOF = 0xA5;
    static constexpr uint16_t NAV_DATA_LEN = (sizeof(NavToEc_s));

    static constexpr uint8_t NAV_DATA_NOERR = 0x00;
    static constexpr uint8_t NAV_DATA_ERR = 0xFF;
    static constexpr uint8_t NAV_NOTARGET = 0xFE;

    Nav();

    static void task(void *_param);
    void sendEcToNavData();
    void incNavRxLost();
    bool isNavOffline();
    NavToEc_s *getNavToEcDataPtr() { return &navToEcDatas_; };

private:
    uint16_t navRxLostCnt_ = 0;
    uint8_t *navRxBuffer_;
    EventGroupHandle_t event_;

    NavToEc_s navToEcDatas_;
    EcToNav_s ecToNavDatas_;
#if NAV_COMM_USB == 1 //&& NAV_COMM_UART == 0
public:
    void navInit(EventGroupHandle_t _event);
    uint8_t parseNavData(uint8_t *_buf, uint16_t _len);

private:
    Usb *navUsb_;
    uint8_t rxBuffer_[NAV_DATA_LEN * 2];
#else
public:
    void navInit(EventGroupHandle_t _event);
    void navUartIdleHandler(UART_HandleTypeDef *_huart);
    uint8_t parseNavData(uint8_t *_buf, uint16_t _len);

private:
    static constexpr uint16_t NAV_RX_BUFFER_LEN = 256;
    static constexpr uint8_t NAV_TX_BUFFER_LEN = 128;
    Uart navUart_;
    uint8_t txBuffer_[NAV_TX_BUFFER_LEN];
    uint16_t lastPos = 0;
    uint16_t dataLen = 0;
    UART_HandleTypeDef NAV_UART_HANDLE = huart1;
    DMA_HandleTypeDef NAV_DMA_HANDLE = hdma_usart1_rx;
#endif
};


} // namespace NAV
// #if APP_USE_NAV
inline NAV::Nav *nav;
// #endif