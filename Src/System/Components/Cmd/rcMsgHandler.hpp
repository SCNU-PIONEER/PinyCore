#pragma once

#include "rc.hpp"
#include "handler.hpp"
#include "msgImpl.hpp"

typedef struct {
    float rx;
    float ry;
    float lx;
    float ly;
} rocker_t;


class rcMsgHandler : public Handler {
public:
    void init(EventGroupHandle_t _event) override;

    void handle() override;

private:
    const float S_CURVE_ACC = 2.0f;
    const uint8_t T_ACC_CNT = 100;

    RC::Rc &rc_ = RC::Rc::instance();
    rocker_t rocker;
};
