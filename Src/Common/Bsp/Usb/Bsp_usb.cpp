#include "Bsp_usb.hpp"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "Soc.hpp"
#include "StmLog.hpp"

#if defined SOC_USB_HS
extern USBD_HandleTypeDef hUsbDeviceHS;
#define USB_DEVICE hUsbDeviceHS
#elif defined SOC_USB_FS
extern USBD_HandleTypeDef hUsbDeviceFS;
#define USB_DEVICE hUsbDeviceFS
#endif

Usb::Usb()
{
    rx_.semphr = xSemaphoreCreateBinary();
    xTaskCreate(task, "Usb", 256, this, 1, nullptr);
}

void usbParse(uint8_t *_data, uint16_t _len)
{
    Usb::instance().parse(_data, _len);
}

void Usb::parse(uint8_t *_data, uint16_t _len)
{
    rx_.buf = _data;
    rx_.len = _len;
    rx_.dataPending = true;
    xSemaphoreGiveFromISR(rx_.semphr, nullptr);
}

int8_t Usb::send(const uint8_t *_data, uint16_t _len)
{
    if (_len > TX_MAX_SIZE) {
        return -1;
    }

#if defined SOC_USB_HS
    uint8_t status = CDC_Transmit_HS((uint8_t *)_data, _len);
#elif defined SOC_USB_FS
    uint8_t status = CDC_Transmit_FS((uint8_t *)_data, _len);
#endif

    if (status != USBD_OK) {
        LOG::error("Usb", "send fucking failed");
        return -1;
    }
    return 0;
}

int8_t Usb::registerCallback(std::function<void(uint8_t *, uint16_t)> _cb)
{
    if (_cb) {
        cbList_.emplace_back(_cb);
        return 0;
    }
    return -1;
}

void Usb::task(void *_arg)
{
    Usb *const pThis = static_cast<Usb *>(_arg);
    for (;;) {
        if (xSemaphoreTake(pThis->rx_.semphr, portMAX_DELAY) == pdTRUE) {
            if (pThis->rx_.dataPending) {
                pThis->rx_.dataPending = false;
                for (auto &cb : pThis->cbList_) {
                    if (cb) {
                        cb(pThis->rx_.buf, pThis->rx_.len);
                    }
                }
                USBD_CDC_ReceivePacket(&USB_DEVICE); // Allow next packet
            }
        }
    }
}
