#pragma once

#include <cstdint>
#include "sdkconfig.h"
#include "Bsp.hpp"
#include "event_groups.h"
#include "MsgImpl.hpp"


namespace MINIPC {

class MiniPc {
public:
    static constexpr int MAX_TX_NUM = 3;
    MiniPc() = default;

    virtual ~MiniPc() = default;
    virtual void init(EventGroupHandle_t _event) = 0;
    virtual void usbIdleCallback(uint8_t *_buf, uint16_t _len) = 0;
    virtual void parseData() = 0;
    virtual void sendData() = 0;

    EventGroupHandle_t event_;
};

} // namespace MINIPC
