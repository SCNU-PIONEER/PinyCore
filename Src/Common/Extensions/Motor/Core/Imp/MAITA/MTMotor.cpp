#include "MTMotor.hpp"
#include "MTMotorMsg.hpp"
#include "StmLog.hpp"
#include "Bsp_can.hpp"
#include "MotorCommonMacros.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>

using namespace PINYMOTOR;
using namespace MTMOTOR;

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

MTMotor::MTMotor(const char _name[16], InitConfig_s _config, WorkMode_e _workMode)
        : IMotor(_name, _config), workMode_(_workMode), rxStream_(xMessageBufferCreate(16))
{
    this->cmd_.clear();
    errCode_.clear();
}

MTMotor::~MTMotor() { this->cancelMotor(); }

void MTMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

bool MTMotor::isEnable() const { return this->cmd_.SW; }

void MTMotor::registerRecvCallback(uint16_t _rxId)
{
    Can::instance().registerCallback(reinterpret_cast<canHandle *>(regInfo_.pComHandle), _rxId,
                                     [this](const uint8_t *_rxBuf) {
                                         xMessageBufferSendFromISR(this->rxStream_, (void *)_rxBuf, 8, nullptr);
                                     });
}
MotorTypeDef_e MTMotor::ctrl()
{
    std::array<uint8_t, 8> txBuf{};
    if (this->cmd_.SW) { //cmd_.SW == true
        (this->*convert)(txBuf);
    } else if (!this->cmd_.SW && this->cmd_.prevSW) { //SW == false && prevSW == true
        disable(txBuf);
    } else if (!this->cmd_.SW && !this->cmd_.prevSW) { //SW == false && prevSW == false
        readState2(txBuf);
    }

    return send(ctrlId_, txBuf, 8);
}

void MTMotor::readErrCode() { this->convert = &MTMotor::readErrorCode; }

//电机手册就图一乐,真设置零点还得读上位机,如未成功请多次reset
void MTMotor::setMotorZeroAng()
{
    setFunctionCtrlData(0x92, 0, 0);
    setFunctionCtrlData(0x64, 0, 0);
    setFunctionCtrlData(0x76, 0, 0);
    setFunctionCtrlData(0x80, 0, 0);
    setFunctionCtrlData(0x92, 0, 0);
    setFunctionCtrlData(0x9A, 0, 0);
}

//设置函数,必须在电机失能时调用
void MTMotor::setFunctionCtrlData(uint8_t _header, uint8_t _index, uint32_t _data)
{
    TransmitFunctionCtrlMsg_s data{};
    std::array<uint8_t, 8> txBuf{};
    data.header = _header;
    data.index = _index;
    data.para = _data;
    memcpy(txBuf.data(), &data, 8);
    send(ctrlId_, txBuf, 8);
}

MotorTypeDef_e MTMotor::parse(const uint8_t *_rxBuf)
{
    if (_rxBuf[0] == 0x9c || _rxBuf[0] == 0xA1 || _rxBuf[0] == 0xA2 || _rxBuf[0] == 0xA4 || _rxBuf[0] == 0xA6 ||
        _rxBuf[0] == 0xA8 || _rxBuf[0] == 0xA9) {
        return parseFeedbackData(_rxBuf);
    } else if (_rxBuf[0] == 0x9A) {
        return parseErrorCode(_rxBuf);
    }
    return 0;
}
//脉塔反馈的errCode包含了所有错误···
MotorTypeDef_e MTMotor::parseErrorCode(const uint8_t *_rxBuf)
{
    auto allErrCode = static_cast<uint16_t>(_rxBuf[6] | (_rxBuf[7] << 8));
    const ErrorCode_e allErrors[] = { ErrorCode_e::STALL_MOTOR,     ErrorCode_e::LOW_VOLT,
                                      ErrorCode_e::OVER_VOLT,       ErrorCode_e::OVER_CURRENT,
                                      ErrorCode_e::POWER_OVER,      ErrorCode_e::PARA_ERR,
                                      ErrorCode_e::OVER_SPEED,      ErrorCode_e::PCB_HIGH_TEMP,
                                      ErrorCode_e::MOTOR_HIG_TEMP,  ErrorCode_e::ENCODER_CAIL_ERR,
                                      ErrorCode_e::ENCDOER_DATA_ERR };
    const char *errCodeStr[] = { "STALL_MOTOR",    "LOW_VOLT",         "OVER_VOLT",       "OVER_CURRENT",
                                 "POWER_OVER",     "PARA_ERR",         "OVER_SPEED",      "PCB_HIGH_TEMP",
                                 "MOTOR_HIG_TEMP", "ENCODER_CAIL_ERR", "ENCDOER_DATA_ERR" };
    for (size_t i = 0; i < sizeof(allErrors) / sizeof(allErrors[0]); i++) {
        errCode_[allErrors[i]] = (allErrCode & static_cast<uint16_t>(allErrors[i])) != 0;
        if (errCode_[allErrors[i]]) {
            LOG::error("MaiTaMotorError:", " %s", errCodeStr[i]);
        }
    }
    return 0;
}

MotorTypeDef_e MTMotor::parseFeedbackData(const uint8_t *_rxBuf)
{
    FbData_s fb = *(FbData_s<int16_t> *)_rxBuf;
    this->data_.tempture = fb.temperature;
    this->data_.curr = regInfo_.isReverse ? -static_cast<float>(fb.iq) * 0.01f : static_cast<float>(fb.iq) * 0.01f;
    this->data_.torq = this->data_.curr * status_.kn;
    /* speed */
    float noumenaVel = deg2rad(static_cast<float>(fb.speed));
    this->data_.spdRadps = regInfo_.isReverse ? -noumenaVel : noumenaVel;
    this->data_.spdRpm = radps2rpm(this->data_.spdRadps);
    /* angle 脉塔pos反馈多圈值,反馈无上限,会溢出*/
    this->data_.rawAng = rangeMap(deg2rad(static_cast<float>(fb.pos)), -PI, PI);
    float noumenaAng = this->data_.rawAng;
    this->data_.ang = regInfo_.isReverse ? -noumenaAng : noumenaAng - this->data_.zeroAng;
    this->data_.singleCirAng = rangeMap(this->data_.ang / this->rr(), -PI, PI);
    if (this->globalState == GlobalState_e::OFFLINE || this->globalState == GlobalState_e::UNRECOGNIZED) {
        this->globalState = GlobalState_e::ONLINE;
        this->data_.angLast = this->data_.ang;
    }
    float delta = this->data_.ang - this->data_.angLast;
    if (delta > PI) {
        this->data_.cirNum -= 1.f / this->rr();
    } else if (delta < -PI) {
        this->data_.cirNum += 1.f / this->rr();
    }
    this->data_.multipCirAng = this->data_.singleCirAng + (TWO_PI * this->data_.cirNum);
    this->data_.angLast = this->data_.ang;
    return 0;
}

MotorTypeDef_e MTMotor::send(uint16_t _sendId, std::array<uint8_t, 8> _txBuf, uint8_t _len)
{
    return Can::instance().transmitData(reinterpret_cast<canHandle *>(regInfo_.pComHandle), _sendId, _txBuf.data(),
                                        _len);
}

void MTMotor::disable(std::array<uint8_t, 8> &_txBuf)
{
    this->cmd_.updateSW(false);
    constexpr std::array<uint8_t, 8> PACK = { 0x80, 0, 0, 0, 0, 0, 0, 0 };
    _txBuf = PACK;
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

void MTMotor::overrideReductionRatio(float _newReductionRatio)
{
    regInfo_.model.reductionRatio = _newReductionRatio;
    status_.torqMax *= _newReductionRatio;
    status_.kn *= _newReductionRatio;
}

bool MTMotor::isSupportMode(WorkMode_e _mode) const
{
    switch (_mode) {
    case WorkMode_e::SETTING:
    case WorkMode_e::TORQ:
    case WorkMode_e::SPEED:
    case WorkMode_e::ABS_POS:
    case WorkMode_e::SINGLE_POS:
    case WorkMode_e::INC_POS:
    case WorkMode_e::FORCE_POS:
        return true;
    default:
        return false;
    }
}
