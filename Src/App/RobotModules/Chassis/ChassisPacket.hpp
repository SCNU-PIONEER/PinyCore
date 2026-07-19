#include "Bsp_can.hpp"
#include "RefereeProt.hpp"
#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "PacketImpl.hpp"

class ChassisCtrlRxPacket : public COMM::RxPacket<COMM::ChassisCtrlPacketType, 1> {
public:
    ChassisCtrlRxPacket() { registerCallback(); }
    void registerCallback() final
    {
        Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
            xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
        });
        LOG::info("ChassisCtrlRxPacket", " Receive cb registered, uId:0x%hx", uid());
    };
};

class RefereeCtrlTxPacket : public COMM::TxPacket<COMM::RefereePacketType> {
public:
    RefereeCtrlTxPacket(float _txFreq) : COMM::TxPacket<COMM::RefereePacketType>(_txFreq) {}
    void transmit(Data &_data) final { Can::instance().transmitData(&HCAN3, uid(), _data.bytes, COMM::RefereePacketType::LEN); }
};

class BulletSpdTxPacket : public COMM::TxPacket<COMM::SpeedMsgPacket> {
public:
    BulletSpdTxPacket(float _txFreq) : COMM::TxPacket<COMM::SpeedMsgPacket>(_txFreq) {}
    void transmit(Data &_data) final { Can::instance().transmitData(&HCAN3, uid(), _data.bytes, COMM::SpeedMsgPacket::LEN); }
};

class DecesionTxPacket1 : public COMM::TxPacket<COMM::DecesionPacket1> {
public:
    DecesionTxPacket1(float _txFreq) : COMM::TxPacket<COMM::DecesionPacket1>(_txFreq) {}
    void transmit(Data &_data) final { Can::instance().transmitData(&HCAN3, uid(), _data.bytes, COMM::DecesionPacket1::LEN); }
};

class DecesionTxPacket2 : public COMM::TxPacket<COMM::DecesionPacket2> {
public:
    DecesionTxPacket2(float _txFreq) : COMM::TxPacket<COMM::DecesionPacket2>(_txFreq) {}
    void transmit(Data &_data) final { Can::instance().transmitData(&HCAN3, uid(), _data.bytes, COMM::DecesionPacket2::LEN); }
};

class SentryInfoTxPacket : public COMM::TxPacket<COMM::SentryInfoPacket> {
public:
    SentryInfoTxPacket(float _txFreq) : COMM::TxPacket<COMM::SentryInfoPacket>(_txFreq) {}
    void transmit(Data &_data) final { Can::instance().transmitData(&HCAN3, uid(), _data.bytes, COMM::SentryInfoPacket::LEN); }
};

class SentryCmdRxPacket : public COMM::RxPacket<COMM::SentryCmdPacket, 1> {
public:
    SentryCmdRxPacket() { registerCallback(); }
    void registerCallback() final
    {
        Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
            xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
        });
        LOG::info("ChassisCtrlRxPacket", " Receive cb registered, uId:0x%hx", uid());
    };
};
