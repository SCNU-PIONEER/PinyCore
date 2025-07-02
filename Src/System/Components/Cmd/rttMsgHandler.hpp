#pragma once
#include <stdint.h>
#include "handler.hpp"
#include "msgImpl.hpp"

#define RTT_NO_ERROR    0
#define RTT_MSG_ERR     0xFE

#define RTT_READY_EVENT (1 << 0)

class RTTMsgHandler : public Handler {
public:
    RTTMsgHandler() {}

    void init(EventGroupHandle_t _event) override;
    void addObserver(IObserver *observer) override;
    void handle() override;
    void notify(Msg &_msg) override;

protected:
    static void parse(TimerHandle_t xTimer);

private:
    uint8_t data[25] = { 0 };

    chassisMsg cmsg;
    gimbalMsg gmsg;
};
