#pragma once
#include "Bsp.hpp"
#include "Handler.hpp"
#include <memory>
#include "RemoteControl.hpp"

#define IS_KEY_PRESS(CODE, KEY) (((CODE) & (KEY)) == (KEY))

enum class RCDevType_e : uint8_t {
    DT7 = 0,
    ET08A = 1,
    VT13 = 2,
};

template <RCDevType_e DEV_TYPE> class RcMsgHandler final : public Handler {
public:
    RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final;
    void handle() final;
    void notify(Msg *_msg, QueueHandle_t _queue) final;

private:
    MsgBus_s *msgBus_;

    ChassisMsg_s cMsg_ = {}; // sample

    std::unique_ptr<RemoteControl> rc_ = nullptr;

    void createRcDev(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);

    void scheduleDaemon(const char *_format);

    void masterHandle();
};
