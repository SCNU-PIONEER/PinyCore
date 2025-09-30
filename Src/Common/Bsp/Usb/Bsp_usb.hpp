#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void usbParse(uint8_t *_data, uint16_t _len);

#ifdef __cplusplus
}

#include "BspBase.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include <functional>
#include <vector>

class Usb : public BspBase<Usb> {
public:
    void parse(uint8_t *_data, uint16_t _len);
    int8_t send(const uint8_t *_data, uint16_t _len);
    int8_t registerCallback(std::function<void(uint8_t *, uint16_t)> _cb);

private:
    friend class BspBase<Usb>;

    static constexpr uint8_t TX_MAX_SIZE = 64;
    static constexpr uint8_t TX_TIMEOUT_MS = 10;

    struct RxStats_s {
        uint32_t cnt = 0;
        uint8_t *buf = nullptr;
        uint32_t len = 0;
        bool dataPending = false;
        SemaphoreHandle_t semphr;
    };

    Usb();
    static void task(void *_arg);

    std::vector<std::function<void(uint8_t *, uint16_t)> > cbList_;
    RxStats_s rx_;
};

#endif
