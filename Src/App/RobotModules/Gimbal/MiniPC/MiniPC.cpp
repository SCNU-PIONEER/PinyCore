#include "MiniPC.hpp"
#include "Bsp_usb.hpp"

using namespace GIMBAL;

MiniPC::MiniPC() : TxPacket<EcToMiniPCPacket>(1000.f) { registerCallback(); }

void MiniPC::transmit(EcToMiniPCPacket::Data_u &_data)
{
    if (Usb::instance().send(_data.bytes, EcToMiniPCPacket::LEN) == 0)
        pcState = PCState_e::ONLINE;
    else
        pcState = PCState_e::OFFLINE;
}

void MiniPC::registerCallback()
{
    Usb::instance().registerCallback([this](uint8_t *_buf, uint16_t _len) {
        if (_len == sizeof(RxPacket::Data)) {
            xQueueSendFromISR(this->queue_, _buf, nullptr);
        }
    });
}
