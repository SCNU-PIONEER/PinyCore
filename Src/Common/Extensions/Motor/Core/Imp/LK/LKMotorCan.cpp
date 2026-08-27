#include "LKMotorCan.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

#include "Bsp_can.hpp"

#include <algorithm>
#include <cstdint>

using namespace PINYMOTOR;
using namespace LKMOTOR;

static constexpr char TAG[] = "LKMotorCAN";

LKMotorCAN::LKMotorCAN(const char _name[16], InitConfig_s _config, WorkMode_e _workMode)
        : IMotor(_name, _config), workMode_(_workMode)
{
    this->cmd_.clear();
    this->cmd_.SW = true;
    cmd(MotorCmdType_e::OFF);

    LOG::CHECK([_name, _config] {
        if (_config.offsetId > 32) {
            LOG::error(TAG, "%s: Invalid offsetId %d. Must be between 0 and 32.", _name, _config.offsetId);
            return STM_ERR_INVALID_ARG;
        }
        if (_config.comType != ComType_e::CAN) {
            LOG::error(TAG, "%s: Only CAN communication supported", _name);
            return STM_ERR_INVALID_ARG;
        }
        return STM_OK;
    });

    AUX_.rxQueue = xQueueCreate(4, sizeof(RxBus_s::CANRxBuf_s<8>::data));
    this->updateCtrlMode();
    this->registerRecvCallback(this->canId());
}

LKMotorCAN::~LKMotorCAN()
{
    this->cancelRecvCallback(this->canId());
    this->cancelMotor();
}

void LKMotorCAN::overrideStats(const Status_s &_stats) { status_ = _stats; }

bool LKMotorCAN::isEnable() const { return this->cmd_.SW; }

uint16_t LKMotorCAN::canId() const { return TX_BASE_ID + regInfo_.offsetId; }

void LKMotorCAN::overrideReductionRatio(float _newReductionRatio)
{
    regInfo_.model.reductionRatio = _newReductionRatio;
    status_.torqueMax *= _newReductionRatio;
    status_.torqueConstant *= _newReductionRatio;
    LOG::info(TAG, "%s: Reduction ratio updated to %.2f", regInfo_.name, _newReductionRatio);
}

void LKMotorCAN::registerRecvCallback(uint16_t _rxId)
{
    Can::instance().registerCallback(reinterpret_cast<canHandle *>(regInfo_.pComHandle), _rxId,
                                     [this](const uint8_t *_rxBuf) {
                                         BaseType_t higherPriorityTaskWoken = pdFALSE;
                                         xQueueSendFromISR(AUX_.rxQueue, _rxBuf, &higherPriorityTaskWoken);
                                     });
    LOG::info(TAG, "%s: Receive cb registered, rxId:0x%hx", regInfo_.name, _rxId);
}

void LKMotorCAN::cancelRecvCallback(uint16_t _rxId)
{
    Can::instance().unregisterCallback(reinterpret_cast<canHandle *>(regInfo_.pComHandle), _rxId);
    LOG::info(TAG, "%s: Receive cb canceled, rxId:0x%hx", regInfo_.name, _rxId);
}

MotorTypeDef_e LKMotorCAN::send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len)
{
    //TODO: bug怎么确保只发送一次的命令发送成功
    // if (this->checkSend()) {
    return static_cast<MotorTypeDef_e>(
            Can::instance().transmitData(reinterpret_cast<canHandle *>(regInfo_.pComHandle), _sendId, _txBuf, _len));
    // }
    return 0;
}

MotorTypeDef_e LKMotorCAN::parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf)
{
    const uint8_t *data = _rxBuf.data;
    MotorTypeDef_e rslt = STM_OK;

    switch (data[0]) {
    case 0x9C:
    case 0xA0:
    case 0xA1:
    case 0xA2:
    case 0xA3:
    case 0xA4:
    case 0xA5:
    case 0xA6:
    case 0xA7:
    case 0xA8:
        rslt |= parseState2(data);
        break;
    case 0x80:
    case 0x88:
    case 0x81:
    case 0x93:
        break;
    case 0x9A:
    case 0x9B:
        rslt |= parseState1andError(data);
        break;
    case 0xC0:
    case 0xC1:
        rslt |= parseCtrlCmd(data);
        break;
    case 0x90:
        rslt |= parseEncoder(data);
        break;
    case 0x19:
        rslt |= parsePosZero(data);
        break;
    case 0x92:
        rslt |= parseMultiPos(data);
        break;
    case 0x94:
        rslt |= parseSinglePos(data);
        break;
    case 0x95:
        rslt |= parseSetPos(data);
        break;
    default:
        break;
    }

    return rslt;
}

MotorTypeDef_e LKMotorCAN::parseState1andError(const uint8_t *_data)
{
    const State1_s *fb = reinterpret_cast<const State1_s *>(_data + 1);

    data_.tempture = fb->temperature;
    data_.curr = regInfo_.isReverse ? -static_cast<float>(fb->current) * 0.01f :
                                      static_cast<float>(fb->current) * 0.01f;
    data_.torq = data_.curr * status_.torqueConstant;
    errorState_ = fb->errorState;

    return STM_OK;
}

MotorTypeDef_e LKMotorCAN::parseState2(const uint8_t *_data)
{
    const State2_s *fb = reinterpret_cast<const State2_s *>(_data + 1);

    data_.tempture = fb->temperature;
    data_.tempture = fb->temperature; // 1°C/1LSB
    // current in A, (66/4096 A) / LSB, for MG motor;(33/4096 A) / LSB, for MF motor
    data_.curr = (float)(fb->current) / 4096.f * (float)this->status_.CurrMax;
    data_.torq = data_.curr * this->status_.torqueConstant;
    data_.spdRadps = deg2rad(static_cast<float>(fb->speed)); // 反馈输出轴速度，1dps/LSB
    data_.spdRpm = radps2rpm(data_.spdRadps);
    /*
     * 双编码器反馈的是输出轴的编码器数据
     * 单编码器反馈的是电机轴的编码器数据（减速前）
     * 14bit encoder range: 0-16383, 15bit encoder range: 0-32767, 16bit encoder range: 0-65535
     */
    float noumenaAng = static_cast<float>(fb->encoder) / this->span() * 2.f * PI;
    data_.rawAng = regInfo_.isReverse ? (2.f * PI) - noumenaAng : noumenaAng;
    data_.ang = data_.rawAng;

    /*
     * 电机内部的单圈认定范围是[-PI, PI], 如果超过范围，会将当前角度设置为0
     */
    data_.singleCirAng = rangeMap(data_.ang / this->rr(), -PI, PI);

    /* 
     * 当电机断电，状态为离线状态，上电第一刻先赋值 angLast
     */
    if (this->globalState == GlobalState_e::OFFLINE || this->globalState == GlobalState_e::UNRECOGNIZED) {
        this->globalState = GlobalState_e::ONLINE;
        data_.angLast = data_.ang;
    }

    /*
     * 在零度附近编码器会在0和6.28之间跳变，而多圈位置控制2是支持控制正负的，所以通过delta来判断编码器是否跨过零度，并计算圈数
     * 1. 当编码器从0跳变到6.28时，delta会大于PI，说明电机反向跨过零度，圈数减1，并且多圈角度等于编码器值减去2PI
     * 2. 当编码器从6.28跳变到0时，delta会小于-PI，说明电机正向跨过零度，圈数加1，并且多圈角度等于编码器值加上2PI
     * 3. 当编码器在零度附近跳变，多圈角通过抵消从而不会跳变
     */
    float delta = data_.ang - data_.angLast;
    if (delta > PI) {
        this->data_.cirNum -= 1.f / this->rr();
    } else if (delta < -PI) {
        this->data_.cirNum += 1.f / this->rr();
    }

    data_.multipCirAng = data_.singleCirAng + (TWO_PI * data_.cirNum);
    data_.angLast = data_.ang;

    return STM_OK;
}

MotorTypeDef_e LKMotorCAN::parseCtrlCmd(const uint8_t *_data)
{
    const uint8_t controlParamID = _data[1];

    if (controlParamID == static_cast<uint8_t>(ParamID_e::ANGLE_PID) ||
        controlParamID == static_cast<uint8_t>(ParamID_e::SPEED_PID) ||
        controlParamID == static_cast<uint8_t>(ParamID_e::CURRENT_PID)) {
        uint16_t kp, ki, kd;
        memcpy(&kp, _data + 2, 2);
        memcpy(&ki, _data + 4, 2);
        memcpy(&kd, _data + 6, 2);
    } else if (controlParamID == static_cast<uint8_t>(ParamID_e::INPUT_TORQUE_LIMIT)) {
        uint16_t torqueLimit;
        memcpy(&torqueLimit, _data + 2, 2);
    } else if (controlParamID == static_cast<uint8_t>(ParamID_e::INPUT_SPEED_LIMIT)) {
        int32_t speedLimit;
        memcpy(&speedLimit, _data + 2, 4);
    } else if (controlParamID == static_cast<uint8_t>(ParamID_e::INPUT_ANGLE_LIMIT)) {
        int32_t angleLimit;
        memcpy(&angleLimit, _data + 2, 4);
    } else if (controlParamID == static_cast<uint8_t>(ParamID_e::INPUT_CURRENT_RAMP)) {
        int32_t currentRamp;
        memcpy(&currentRamp, _data + 2, 4);
    } else if (controlParamID == static_cast<uint8_t>(ParamID_e::INPUT_SPEED_RAMP)) {
        int32_t speedRamp;
        memcpy(&speedRamp, _data + 2, 4);
    } else {
        return STM_FAIL;
    }
    return STM_OK;
}

MotorTypeDef_e LKMotorCAN::parseEncoder(const uint8_t *_data)
{
    uint16_t encoder, encoderRaw, encoderOffset;
    memcpy(&encoder, _data + 2, 2);
    memcpy(&encoderRaw, _data + 4, 2);
    memcpy(&encoderOffset, _data + 6, 2);

    return STM_OK;
}

MotorTypeDef_e LKMotorCAN::parsePosZero(const uint8_t *_data)
{
    uint16_t posZero;
    memcpy(&posZero, _data + 4, 2);

    return STM_OK;
}

MotorTypeDef_e LKMotorCAN::parseMultiPos(const uint8_t *_data)
{
    // CAN 帧只有 8 字节, data[1..7] = 7 字节, int64_t 需要 8 字节
    // 高字节缺失, 需要符号扩展
    int64_t pos = 0;
    memcpy(&pos, _data + 1, 7);
    if (_data[7] & 0x80) {
        pos |= static_cast<int64_t>(0xFF) << 56;
    }

    this->data_.multipCirAng = deg2rad(static_cast<float>(pos) * 0.01f);
    this->data_.cirNum = this->data_.multipCirAng / (2.f * PI);

    return STM_OK;
}

MotorTypeDef_e LKMotorCAN::parseSinglePos(const uint8_t *_data)
{
    uint32_t pos;
    memcpy(&pos, _data + 4, 4);

    float noumenaAng = static_cast<float>(pos) / status_.innerReductionRatio;
    noumenaAng = deg2rad(noumenaAng * 0.01f);
    noumenaAng = regInfo_.isReverse ? (2.f * PI) - noumenaAng : noumenaAng;
    this->data_.rawAng = noumenaAng;
    this->data_.ang = this->data_.rawAng;
    this->data_.angLast = this->data_.rawAng;
    this->data_.singleCirAng = this->data_.rawAng;

    return STM_OK;
}

MotorTypeDef_e LKMotorCAN::parseSetPos(const uint8_t *_data)
{
    int32_t pos;
    memcpy(&pos, _data + 4, 4);

    this->data_.multipCirAng = deg2rad(static_cast<float>(pos) * 0.01f);

    return STM_OK;
}

MotorTypeDef_e LKMotorCAN::ctrl()
{
    MotorTypeDef_e rslt = 0;
    TxBus txBuf = {};

    if (this->cmd_.SW && !this->cmd_.prevSW) {
        constexpr uint8_t ENABLE_CMD[8] = { 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        memcpy(txBuf.data, ENABLE_CMD, 8);
        txBuf.len = 8;
    } else if (!this->cmd_.SW) {
        constexpr uint8_t DISABLE_CMD[8] = { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        memcpy(txBuf.data, DISABLE_CMD, 8);
        txBuf.len = 8;
    } else {
        (this->*convert)(txBuf);
    }

    if (txBuf.len > 0) {
        rslt |= this->send(this->ctrlId_, txBuf.data, txBuf.len);
    }
    return rslt;
}

MotorTypeDef_e LKMotorCAN::update()
{
    uint8_t rxData[8];
    if (xQueueReceive(AUX_.rxQueue, rxData, 0) == pdTRUE) {
        AUX_.recvCnt++;
        RxBus_s::CANRxBuf_s<8> rxBuf;
        memcpy(rxBuf.data, rxData, 8);
        this->parse(rxBuf);
    }

    taskENTER_CRITICAL();
    this->parseCmd();
    taskEXIT_CRITICAL();

    this->calcRecvFreq();
    MotorTypeDef_e rslt = ctrl();
    return rslt;
}

MotorTypeDef_e LKMotorCAN::txConvert(const uint8_t _cmdid)
{
    uint8_t txBuf[8] = { _cmdid, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    return this->send(this->ctrlId_, txBuf, 8);
}
MotorTypeDef_e LKMotorCAN::readState1andError() { return txConvert(0x9A); }

MotorTypeDef_e LKMotorCAN::cleanError() { return txConvert(0x9B); }

MotorTypeDef_e LKMotorCAN::readState2() { return txConvert(0x9C); }

MotorTypeDef_e LKMotorCAN::readState3() { return txConvert(0x9D); }

MotorTypeDef_e LKMotorCAN::disable() { return txConvert(0x80); }

MotorTypeDef_e LKMotorCAN::enable() { return txConvert(0x88); }

MotorTypeDef_e LKMotorCAN::stop() { return txConvert(0x81); }

MotorTypeDef_e LKMotorCAN::readEncoder() { return txConvert(0x90); }

MotorTypeDef_e LKMotorCAN::setZeroPos() { return txConvert(0x19); }

MotorTypeDef_e LKMotorCAN::readMultiPos() { return txConvert(0x92); }

MotorTypeDef_e LKMotorCAN::clearPosCircle() { return txConvert(0x93); }

MotorTypeDef_e LKMotorCAN::readSinglePos() { return txConvert(0x94); }

MotorTypeDef_e LKMotorCAN::readCtrlCmd(const ParamID_e _id)
{
    uint8_t txBuf[8] = { 0xC0, static_cast<uint8_t>(_id), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    return this->send(this->ctrlId_, txBuf, 8);
}

MotorTypeDef_e LKMotorCAN::writeCtrlCmd(const ParamID_e _id, const std::array<uint8_t, 6> _data)
{
    uint8_t txBuf[8];
    txBuf[0] = 0xC1;
    txBuf[1] = static_cast<uint8_t>(_id);
    memcpy(txBuf + 2, _data.data(), 6);
    return this->send(this->ctrlId_, txBuf, 8);
}

MotorTypeDef_e LKMotorCAN::setPos(const float _angle)
{
    uint8_t txBuf[8] = { 0x95, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const int32_t angleControl = (int32_t)(rad2deg(_angle) * 100);
    memcpy(txBuf + 4, &angleControl, 4);
    return this->send(this->ctrlId_, txBuf, 8);
}

void LKMotorCAN::convertVOLT(TxBus &_txBuf)
{
    VOLTMsg_s msg = {};
    msg.powerControl = std::clamp((int16_t)(this->cmd_.elec), (int16_t)-850, (int16_t)850);
    memcpy(_txBuf.data, &msg, 8);
    _txBuf.len = 8;
}

void LKMotorCAN::convertMIT(TxBus &_txBuf)
{
    MITMsg_s msg = {};
    // current in A, (66/4096 A) / LSB, for MG motor;(33/4096 A) / LSB, for MF motor
    float iqFloat = this->cmd_.torq / status_.torqueConstant;
    this->cmd_.elec = iqFloat / (float)status_.CurrMax * 4096.f;
    int16_t iqControl = std::clamp(static_cast<int16_t>(this->cmd_.elec), (int16_t)-2048, (int16_t)2048);
    msg.iqControl = iqControl;
    memcpy(_txBuf.data, &msg, 8);
    _txBuf.len = 8;
}

void LKMotorCAN::convertVDES(TxBus &_txBuf)
{
    VDESMsg_s msg = {};
    // 输入力矩做安全保护，将力矩转为电流发送
    const float iqFloat = this->cmd_.torq / status_.torqueConstant;
    const float iqControl = iqFloat / (float)status_.CurrMax * 4096.f;
    msg.iqControl = std::clamp((int16_t)iqControl, (int16_t)-2048, (int16_t)2048);
    const float speed = std::clamp(this->cmd_.vel, (float)-this->status_.speedMax, (float)this->status_.speedMax);
    msg.speedControl = (int32_t)(rad2deg(speed) * 100); //0.01dps/LSB
    memcpy(_txBuf.data, &msg, 8);
    _txBuf.len = 8;
}

void LKMotorCAN::convertMULTIPDES(TxBus &_txBuf)
{
    MULTIPDESMsg_s msg = {};
    msg.angleControl = static_cast<int32_t>(rad2deg(this->cmd_.pos) * 100);
    memcpy(_txBuf.data, &msg, 8);
    _txBuf.len = 8;
}
/**
 * @brief 减速前控制
 * @note 断控前的位置必须保证在[-PI,PI],否则电机内部识别为过圈，并且将当前角度为0
 * @return MotorTypeDef_e
 */
void LKMotorCAN::convertMULTIPDESVDES(TxBus &_txBuf)
{
    MULTIPDESVDESMsg_s msg = {};

    float speed;
    if (this->cmd_.vel == 0) {
        speed = 0.01f; // if speed is zero, maxSpeed will be set MAX
    } else {
        speed = this->cmd_.vel;
    }
    msg.maxSpeed = (uint16_t)(rad2deg(speed) * 100); // 0.01dps/LSB

    msg.angleControl = (int32_t)(rad2deg(this->cmd_.pos) * this->status_.innerReductionRatio *
                                 100); // 0.01degree/LSB, 控制为减速前
    memcpy(_txBuf.data, &msg, 8);
    _txBuf.len = 8;
}

void LKMotorCAN::convertSINGLEPDES(TxBus &_txBuf)
{
    SINGLEPDESMsg_s msg = {};
    msg.spinDirection = this->regInfo_.isReverse;
    msg.angleControl = static_cast<uint32_t>(rad2deg(rangeMap(this->cmd_.pos)) * 100);
    memcpy(_txBuf.data, &msg, 8);
    _txBuf.len = 8;
}

void LKMotorCAN::convertSINGLEPDESVDES(TxBus &_txBuf)
{
    SINGLEPDESVDESMsg_s msg = {};
    msg.spinDirection = this->regInfo_.isReverse;
    const float angle = rangeMap(this->cmd_.pos);
    msg.angleControl = (uint32_t)(rad2deg(angle) * 100);      // range:0~35999, 0.01degree/LSB, 0~360 degree
    msg.maxSpeed = (uint16_t)(rad2deg(this->cmd_.vel) * 100); // 0.01dps/LSB
    memcpy(_txBuf.data, &msg, 8);
    _txBuf.len = 8;
}

void LKMotorCAN::convertINCPDES(TxBus &_txBuf)
{
    INCPDESMsg_s msg = {};
    msg.angleIncrement = static_cast<int32_t>(rad2deg(regInfo_.isReverse ? -this->cmd_.pos : this->cmd_.pos) * 100);
    memcpy(_txBuf.data, &msg, 8);
    _txBuf.len = 8;
}

void LKMotorCAN::convertINCPDESVDES(TxBus &_txBuf)
{
    INCPDESVDESMsg_s msg = {};
    msg.angleIncrement =
            (int32_t)(rad2deg(regInfo_.isReverse ? -this->cmd_.pos : this->cmd_.pos) * 100); // 0.01degree/LSB
    msg.maxSpeed = (uint16_t)(rad2deg(this->cmd_.vel) * 100);

    memcpy(_txBuf.data, &msg, 8);
    _txBuf.len = 8;
}

void LKMotorCAN::convertDisable(TxBus &_txBuf)
{
    constexpr uint8_t DISABLE_CMD[8] = { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    memcpy(_txBuf.data, DISABLE_CMD, 8);
    _txBuf.len = 8;
}

void LKMotorCAN::switchCtrlMode(WorkMode_e _workMode)
{
    this->workMode_ = _workMode;
    updateCtrlMode();
}

void LKMotorCAN::updateCtrlMode()
{
    if (!isSupportMode(workMode_)) {
        LOG::error(TAG, "%s: unsupported work mode", regInfo_.name);
        convert = &LKMotorCAN::convertDisable;
        ctrlId_ = 0xFFFF;
        return;
    }

    ctrlId_ = this->canId();

    switch (workMode_) {
    case WorkMode_e::VOLT:
        convert = &LKMotorCAN::convertVOLT;
        break;
    case WorkMode_e::MIT_TT:
        convert = &LKMotorCAN::convertMIT;
        break;
    case WorkMode_e::VDES:
        convert = &LKMotorCAN::convertVDES;
        break;
    case WorkMode_e::MULTI_PDES:
        convert = &LKMotorCAN::convertMULTIPDES;
        break;
    case WorkMode_e::MULTI_PDESVDES:
        convert = &LKMotorCAN::convertMULTIPDESVDES;
        break;
    case WorkMode_e::SINGLE_PDES:
        convert = &LKMotorCAN::convertSINGLEPDES;
        break;
    case WorkMode_e::SINGLE_PDESVDES:
        convert = &LKMotorCAN::convertSINGLEPDESVDES;
        break;
    case WorkMode_e::INC_PDES:
        convert = &LKMotorCAN::convertINCPDES;
        break;
    case WorkMode_e::INC_PDESVDES:
        convert = &LKMotorCAN::convertINCPDESVDES;
        break;
    default:
        convert = &LKMotorCAN::convertDisable;
        ctrlId_ = 0xFFFF;
        break;
    }
}

bool LKMotorCAN::isSupportMode(WorkMode_e _mode) const
{
    switch (_mode) {
    case WorkMode_e::VOLT:
    case WorkMode_e::MIT_TT:
    case WorkMode_e::VDES:
    case WorkMode_e::MULTI_PDES:
    case WorkMode_e::MULTI_PDESVDES:
    case WorkMode_e::SINGLE_PDES:
    case WorkMode_e::SINGLE_PDESVDES:
    case WorkMode_e::INC_PDES:
    case WorkMode_e::INC_PDESVDES:
        return true;
    default:
        return false;
    }
}
