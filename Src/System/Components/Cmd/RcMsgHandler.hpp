#pragma once
#include "Rc.hpp"
#include "Handler.hpp"

class rcMsgHandler : public Handler {
    typedef struct {
        float rx;
        float ry;
        float lx;
        float ly;
    } rocker_t;

public:
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) override;
    void handle() override;
    void notify(Msg *_msg, QueueHandle_t _queue) override;

private:
    static constexpr float S_CURVE_ACC = 2.0f;
    static constexpr uint8_t T_ACC_CNT = 100;

    RC::Rc &rc_ = RC::Rc::instance();
    rocker_t rocker;
    MsgBus_s *msgBus;
};
