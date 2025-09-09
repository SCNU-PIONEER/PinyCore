#pragma once
#include "Rc.hpp"
#include "Handler.hpp"

class RcMsgHandler : public Handler {
    static constexpr float ROCKER_VX_GAIN = 2.f;
    static constexpr float ROCKER_VY_GAIN = 2.f;
    static constexpr float ROCKER_WZ_GAIN = 0.06f;

    static constexpr float ROCKER_PITCH_GAIN = 0.1f;
    static constexpr float ROCKER_YAW_GAIN = 0.1f;
    typedef struct {
        struct {
            float rx;
            float ry;
            float lx;
            float ly;
        };

        struct {
            uint8_t rSwitch;
            uint8_t lSwitch;
        };

        struct {
            uint8_t rPress;
            uint8_t lPress;
            int16_t xMove;
            int16_t yMove;
            int16_t zRoller;
        }; // TODO:
    } RcMsg_t;

public:
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) override;
    void handle() override;
    void notify(Msg *_msg, QueueHandle_t _queue) override;

private:
    static constexpr float S_CURVE_ACC = 2.0f;
    static constexpr uint8_t T_ACC_CNT = 100;

    RC::Rc &rc_ = RC::Rc::instance();

    RcMsg_t rcMsg_ = {};

    RcMsg_t rcMsgPrev_ = {};

    MsgBus_s *msgBus_;

    static void updateRocker(float &_target, float _channel);

    void chassisHandle();
    void masterHandle();

    ChassisMsg_s cmsg_;
};
