#pragma once
#include "Rc.hpp"
#include "Handler.hpp"
#include "MsgImpl.hpp"

class rcMsgHandler : public Handler {
    typedef struct {
        float rx;
        float ry;
        float lx;
        float ly;
    } rocker_t;

public:
    void init(EventGroupHandle_t _event) override;
    void addObserver(IObserver *observer) override;
    void handle() override;
    void notify(Msg &_msg) override;

private:
    const float S_CURVE_ACC = 2.0f;
    const uint8_t T_ACC_CNT = 100;

    RC::Rc &rc_ = RC::Rc::instance();
    rocker_t rocker;

    chassisMsg cmsg;
    gimbalMsg gmsg;
};
