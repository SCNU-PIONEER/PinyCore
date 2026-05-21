#include "Bsp_can.hpp"
#include "RefereeProt.hpp"
#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "PacketImpl.hpp"


class ChassisCtrlTxPacket : public COMM::TxPacket<COMM::ChassisCtrlPacketType> {
public:
    ChassisCtrlTxPacket(float _txFreq) : TxPacket(_txFreq) {}
    void transmit(uint8_t *_buf, uint16_t _len) final { Can::instance().transmitData(&HCAN3, uid(), _buf, _len); };
};

class ShooterCtrlTxPacket : public COMM::TxPacket<COMM::ArmorBoosterPacketType> {
public:
    ShooterCtrlTxPacket(float _txFreq) : TxPacket(_txFreq) {}
    void transmit(uint8_t *_buf, uint16_t _len) final { Can::instance().transmitData(&HCAN3, uid(), _buf, _len); };
};

class RefereeRxPacket : public COMM::RxPacket<COMM::RefereePacketType, 1> {
public:
    RefereeRxPacket() { registerCallback(); }
    void registerCallback() final
    {
        Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
            xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
        });
        LOG::info("RefereeRxPacket", " Receive cb registered, uId:0x%hx", uid());
    };
};

class BulletSpdRxPacket : public COMM::RxPacket<COMM::SpeedMsgPacket, 1> {
public:
    BulletSpdRxPacket() { registerCallback(); }
    void registerCallback() final
    {
        Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
            xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
        });
        LOG::info("BulletSpdRxPacket", " Receive cb registered, uId:0x%hx", uid());
    };
};

class DecesionRxPacket1 : public COMM::RxPacket<COMM::DecesionPacket1, 1> {
public:
    DecesionRxPacket1() { registerCallback(); }
    void registerCallback() final
    {
        Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
            xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
        });
    };
};

class DecesionRxPacket2 : public COMM::RxPacket<COMM::DecesionPacket2, 1> {
public:
    DecesionRxPacket2() { registerCallback(); }
    void registerCallback() final
    {
        Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
            xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
        });
    };
};

class SentryInfoRxPacket : public COMM::RxPacket<COMM::SentryInfoPacket, 1> {
public:
    SentryInfoRxPacket() { registerCallback(); }
    void registerCallback() final
    {
        Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
            xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
        });
    };
};

class SentryCmdTxPacket : public COMM::TxPacket<COMM::SentryCmdPacket> {
public:
    SentryCmdTxPacket(float _txFreq) : TxPacket(_txFreq) {}
    void transmit(uint8_t *_buf, uint16_t _len) final { Can::instance().transmitData(&HCAN3, uid(), _buf, _len); }
};
