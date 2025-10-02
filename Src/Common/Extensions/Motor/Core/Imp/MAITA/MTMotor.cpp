#include "MTMotor.hpp"
#include "MTMotorMsg.hpp"
#include "Bsp_can.hpp"
#include "MotorCommonMacros.hpp"
#include <cstring>

using namespace PINYMOTOR;
using namespace MTMOTOR;

static constexpr char TAG[] = "MTMotor";

Status_s &Status_s::operator=(const Status_s &_other)
{
    if (this != &_other) {
        speedMax = _other.speedMax;
        currMax = _other.currMax;
        torqMax = _other.torqMax;
        np = _other.np;
        interRR = _other.interRR;
        kn = _other.kn;
    }
    return *this;
}

MTMotor::MTMotor(const char _name[16], InitConfig_s _config)
        : IMotor(_name, _config), rxStream_(xMessageBufferCreate(16))
{
    this->cmd_.clear();
}

MTMotor::~MTMotor() { this->cancelMotor(); }

void MTMotor::updateCtrlMode()
{
    ctrlId_ = regInfo_.model.txBaseId + regInfo_.offsetId;
}

void MTMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

bool MTMotor::isEnable() const { return this->cmd_.SW; }

void MTMotor::registerRecvCallback(uint16_t _rxId)
{
    Can::instance().registerCallback(
            reinterpret_cast<canHandle *>(regInfo_.pComHandle), _rxId,
            [this](const uint8_t *_rxBuf) {
                xMessageBufferSendFromISR(this->rxStream_, (void *)_rxBuf, 8,
                                          nullptr);
            });
}

MotorTypeDef_e MTMotor::parse(const uint8_t *_rxBuf)
{
    if (_rxBuf[0] == 0xA4 || _rxBuf[0] == 0x9C) {
        return parseAbsPosCtrl(_rxBuf);
    }
    return 0;
}

MotorTypeDef_e MTMotor::parseAbsPosCtrl(const uint8_t *_rxBuf)
{
    Feedback_s fb = *(Feedback_s *)_rxBuf;
    data_.tempture = fb.temperature;
    data_.curr = regInfo_.isReverse ? -static_cast<float>(fb.iq) * 0.01f :
                                      static_cast<float>(fb.iq) * 0.01f;
    data_.torq = data_.curr * status_.kn;
    /* speed */
    float noumenaVel = deg2rad(static_cast<float>(fb.speed));
    this->data_.spdRadps = regInfo_.isReverse ? -noumenaVel : noumenaVel;
    this->data_.spdRpm = radps2rpm(this->data_.spdRadps);
    /* angle */
    this->data_.rawAng = static_cast<float>(fb.pos);
    float noumenaAng = deg2rad(this->data_.rawAng);
    float rawAng = regInfo_.isReverse ? -noumenaAng : noumenaAng;
    float delta = rawAng - this->data_.zeroAng;
    this->data_.angLast = this->data_.ang;
    this->data_.ang = delta;
    this->data_.singleCirAng = rangeMap(this->data_.ang, -PI, PI);
    this->data_.multipCirAng = this->data_.ang;
    this->data_.cirNum = this->data_.multipCirAng / (2.f * PI);
    return 0;
}


MotorTypeDef_e MTMotor::send(uint16_t _sendId, std::array<uint8_t, 8> _txBuf,
                             uint8_t _len)
{
    return Can::instance().transmitData(
            reinterpret_cast<canHandle *>(regInfo_.pComHandle), _sendId,
            _txBuf.data(), _len);
}


MotorTypeDef_e MTMotor::ctrl()
{
    std::array<uint8_t, 8> txBuf{};

    if (this->cmd_.SW) {
        absPosCtrl(txBuf);
    } else if (!this->cmd_.SW && this->cmd_.prevSW) {
        if (this->posPID_ != nullptr)
            this->posPID_->reset();
        if (this->velPID_ != nullptr)
            this->velPID_->reset();
        if (this->torqPID_ != nullptr)
            this->torqPID_->reset();
        disable(txBuf);
    } else if (!this->cmd_.SW && !this->cmd_.prevSW) {
        readState2(txBuf);
    }

    return send(ctrlId_, txBuf, 8);
}


void MTMotor::disable(std::array<uint8_t, 8> &_txBuf)
{
    this->cmd_.updateSW(false);
    constexpr std::array<uint8_t, 8> PACK = { 0x80, 0, 0, 0, 0, 0, 0, 0 };
    _txBuf = PACK;
}

void MTMotor::readState2(std::array<uint8_t, 8> &_txBuf)
{
    constexpr std::array<uint8_t, 8> PACK = { 0x9C, 0, 0, 0, 0, 0, 0, 0 };
    _txBuf = PACK;
}

void MTMotor::absPosCtrl(std::array<uint8_t, 8> &_txBuf)
{
    TransmitMsg_s data{};
    uint16_t rawSpeed = static_cast<uint16_t>(rad2deg(this->cmd_.vel));
    data.maxspeed = std::min(rawSpeed, this->status_.speedMax);
    data.pos = regInfo_.isReverse ?
                       -static_cast<int32_t>(rad2deg(this->cmd_.pos) * 100) :
                       static_cast<int32_t>(rad2deg(this->cmd_.pos) * 100);
    memcpy(_txBuf.data(), &data, 8);
}


MotorTypeDef_e MTMotor::update()
{
    uint8_t rxData[8];
    if (xMessageBufferReceive(rxStream_, rxData, 8, 0)) {
        AUX_.recvCnt++;
        parse(rxData);
    }
    calcRecvFreq();

    taskENTER_CRITICAL();
    this->parseCmd();
    taskEXIT_CRITICAL();

    return ctrl();
}
