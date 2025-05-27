#pragma once
#include <stdint.h>
#include <string.h>
#include "handler.hpp"
#include "SEGGER_RTT.h"

#define RTT_NO_ERROR 0
#define RTT_MSG_ERR  0xFE

class RTTMsgHandler : public Handler {
public:
    RTTMsgHandler(chassisMsg *cmsg, gimbalMsg *gmsg) : Handler(cmsg, gmsg) {}

    inline uint8_t parseData() override
    {
        if (SEGGER_RTT_HasKey()) {
            memset(data, 0, sizeof(data));
            SEGGER_RTT_Read(0, data, sizeof(data) - 1);
            return RTT_NO_ERROR;
        }
        return RTT_MSG_ERR;
    }

    inline void handle() override
    {
        if (strcmp((const char *)data, "run\n") == 0) {
            cmsg->state = State_e::run;
        } else if (strcmp((const char *)data, "stop\n") == 0) {
            cmsg->state = State_e::stop;
        } else {
            cmsg->state = State_e::stop;
        }
    }

private:
    uint8_t data[25] = { 0 };
};
