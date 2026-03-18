#pragma once
#include "Bsp.hpp"
#include "Handler.hpp"
#include <memory>
#include "RemoteControl.hpp"

#define IS_KEY_PRESS(CODE, KEY) (((CODE) & (KEY)) == (KEY))
#define IS_KEY_DT7(KEY)         IS_KEY_PRESS(rcData->keyboard.keyCode, DT7::KEY)
#define IS_LAST_KEY_DT7(KEY)    IS_KEY_PRESS(rcData->keyboard.lastKeyCode, DT7::KEY)

#define IS_KEY_VT13(KEY)      IS_KEY_PRESS(rcData->KBM.key, VT13::KEY)
#define IS_LAST_KEY_VT13(KEY) IS_KEY_PRESS(rcData->KBM.lastKey, VT13::KEY)

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
    static constexpr float INPUT_VX_MAX = 3.f;
    static constexpr float INPUT_VY_MAX = 3.f;
    static constexpr uint8_t T_ACC_CNT = 100;
    static constexpr float S_CURVE_ACC = 3.0f;
    static constexpr float INPUT_YAW_MAX = 3.f;
    static constexpr float GIMBAL_YAW_MAX = 0.002f;
    static constexpr float GAIN_MOUSE_X = 0.0001f;
    static constexpr float GAIN_MOUSE_Y = 0.0001f;

    float channelMax_;

    MsgBus_s *msgBus_;

    ChassisMsg_s cMsg_ = {}; // sample

    std::unique_ptr<RemoteControl> rc_ = nullptr;

    void createRcDev(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);

    void scheduleDaemon();

    void masterHandle();

    void updateRocker(float &_target, float _channel);

    float rx = 0.f;
    float ry = 0.f;
    float lx = 0.f;
    float ly = 0.f;

    float gimbalVal = 0.f;
};
