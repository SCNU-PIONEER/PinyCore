#pragma once
#include "Bsp.hpp"
#include "Handler.hpp"

#define IS_KEY_PRESS(CODE, KEY) (((CODE) & (KEY)) == (KEY))

class RcMsgHandler final : public Handler {
    static constexpr float INPUT_VX_MAX = 4.f;
    static constexpr float INPUT_VY_MAX = 4.f;

    static constexpr float INPUT_PITCH_MAX = 3.f;
    static constexpr float INPUT_YAW_MAX = 7.f;

public:
    RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final;
    void handle() final;
    void notify(Msg *_msg, QueueHandle_t _queue) final;

private:
    MsgBus_s *msgBus_;

    GimbalMsg_s gMsg_ = {};
    ChassisMsg_s cMsg_ = {};
    ArmorLoaderMsg_s alMsg_ = {};
    ArmorBoosterMsg_s abMsg_ = {};

    void masterHandle();
};
