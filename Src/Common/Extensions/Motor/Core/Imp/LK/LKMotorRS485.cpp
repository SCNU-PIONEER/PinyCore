#include "LKMotorRS485.hpp"
#include "StmLog.hpp"
#include "Bsp.hpp"
#include "MotorCommonMacros.hpp"


using namespace PINYMOTOR;
using namespace LKMOTOR;

static constexpr char TAG[] = "LKMotorRS485";

LKMotorRS485::LKMotorRS485(const char _name[16], InitConfig_s _config)
        : IMotor(_name, _config)
        , uart_(reinterpret_cast<UART_HandleTypeDef *>(_config.pComHandle))
        , txBuf_((uint8_t *)Dma::instance().ram_alloc(TXBUF_LEN))
        , rxBuf_((uint8_t *)Dma::instance().ram_alloc(RXBUF_LEN))
{
    this->cmd_.clear();
    this->cmd_.SW = true;     // 默认上电使能 (电机特性)
    cmd(MotorCmdType_e::OFF); // 期望上电失能 (for safe)

    LOG::CHECK([_name, _config] {
        if (_config.offsetId > 32) {
            LOG::error(TAG, "%s: Invalid offsetId %d. Must be between 0 and 32.", _name, _config.offsetId);
            return STM_ERR_INVALID_ARG;
        }
        if (_config.comType != ComType_e::RS485) {
            LOG::error(TAG, "%s: Only RS485 communication supported", _name);
            return STM_ERR_INVALID_ARG;
        }

        if (_config.txFreq > 1000) {
            LOG::error(TAG, "%s: TX freq max 1000Hz (got %.0fHz)", _name, _config.txFreq);
            return STM_ERR_INVALID_ARG;
        }
        if (((UART_HandleTypeDef *)(_config.pComHandle))->Init.BaudRate <= 1000000 && _config.txFreq >= 500) {
            LOG::error(TAG, "%s: txFreq must be <= 500Hz when baud rate < 1Mbps", _name);
            return STM_ERR_INVALID_ARG;
        } else if (((UART_HandleTypeDef *)(_config.pComHandle))->Init.BaudRate <= 115200 && _config.txFreq >= 200) {
            LOG::error(TAG, "%s: txFreq must be <= 200Hz when baud rate < 115200", _name);
            return STM_ERR_INVALID_ARG;
        }
        return STM_OK;
    });

    registerRecvCallback(this->regInfo_.offsetId);
    updateCtrlMode(this->regInfo_.workMode);
    AUX_.rxQueue = xQueueCreate(4, RXBUF_LEN);
    uart_.recvDmaInit(rxBuf_, RXBUF_LEN);
}

LKMotorRS485::~LKMotorRS485() { this->cancelMotor(); }

bool LKMotorRS485::isEnable() const { return this->cmd_.SW; }

void LKMotorRS485::registerRecvCallback(uint16_t _rxId)
{
    uart_.registerCallback([this](UART_HandleTypeDef *_huart, uint16_t _dataLength) {
        xQueueSendFromISR(AUX_.rxQueue, _huart->pRxBuffPtr, nullptr);
    });
}

uint8_t rxbuf[20]{};
MotorTypeDef_e LKMotorRS485::parse()
{
    if (this->rxBuf_[0] != 0x3E) {
        return STM_FAIL;
    }

    memcpy(rxbuf, this->rxBuf_, RXBUF_LEN);

    MotorTypeDef_e rslt = STM_OK;

    switch (this->rxBuf_[1]) {
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
        rslt |= parseState2();
        break;
    case 0x80: // disable command has no response, just return OK
        if (this->globalState == GlobalState_e::OFFLINE || this->globalState == GlobalState_e::UNRECOGNIZED) {
            rslt |= readSinglePos(); // 第一次上电，读取当前位置，避免 angle last 为空
        }
        uart_.receiveDma(rxBuf_, RXBUF_LEN);
        return rslt;
    case 0x88: // enable command has no response, just return OK
    case 0x81: // stop command has no response, just return OK
    case 0x93: // clear pos circle command has no response, just return OK
        break;
    case 0x9A:
    case 0x9B:
        rslt |= parseState1andError();
        break;
    case 0xC0:
    case 0xC1:
        rslt |= parseCtrlCmd();
        break;
    case 0x90:
        rslt |= parseEncoder();
        break;
    case 0x19:
        rslt |= parsePosZero();
        break;
    case 0x92:
        rslt |= parseMultiPos();
        break;
    case 0x94:
        rslt |= parseSinglePos();
        break;
    case 0x95:
        rslt |= parseSetPos();
        break;
    default:
        break;
    }

    uart_.receiveDma(rxBuf_, RXBUF_LEN);

    return rslt;
}

void LKMotorRS485::updateCtrlMode(const WorkMode_e _mode)
{
    switch (_mode) {
    case WorkMode_e::VOLT:
        this->ctrlCallback = &LKMotorRS485::openloopCtrl;
        break;
    case WorkMode_e::MIT_TT:
        this->ctrlCallback = &LKMotorRS485::torqCtrl;
        break;
    case WorkMode_e::VDES:
        this->ctrlCallback = &LKMotorRS485::velCtrl;
        break;
    case WorkMode_e::SINGLE_PDES:
        this->ctrlCallback = &LKMotorRS485::singlePosCtrl1;
        break;
    case WorkMode_e::SINGLE_PDESVDES:
        this->ctrlCallback = &LKMotorRS485::singlePosCtrl2;
        break;
    case WorkMode_e::MULTI_PDES:
        this->ctrlCallback = &LKMotorRS485::multiPosCtrl1;
        break;
    case WorkMode_e::MULTI_PDESVDES:
        this->ctrlCallback = &LKMotorRS485::multiPosCtrl2;
        break;
    case WorkMode_e::INC_PDES:
        this->ctrlCallback = &LKMotorRS485::incrementalPosCtrl1;
        break;
    case WorkMode_e::INC_PDESVDES:
        this->ctrlCallback = &LKMotorRS485::incrementalPosCtrl2;
        break;
    default:
        LOG::error(TAG, " %s: invalid work mode", _mode);
        break;
    }
}

/*
 * @brief check sum
 *
 * @param _data
 * @param _dataLen: data length, not include check data sum
 * @return MotorTypeDef_e
 */
MotorTypeDef_e LKMotorRS485::checkSum(const uint8_t *_data, const uint8_t _dataLen)
{
    uint8_t cmdSum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        cmdSum += _data[i];
    }
    if (cmdSum != _data[4])
        return STM_FAIL;

    if (_dataLen == 0)
        return STM_OK;

    uint8_t dataSum = 0;
    for (uint8_t i = 0; i < _dataLen; i++) {
        dataSum += _data[5 + i];
    }
    return dataSum == _data[5 + _dataLen] ? STM_OK : STM_FAIL;
}

MotorTypeDef_e LKMotorRS485::parseState1andError()
{
    if (checkSum(this->rxBuf_, 7))
        return STM_FAIL;

    State1_s *fb = (State1_s *)(this->rxBuf_ + 5);
    data_.tempture = fb->temperature; // 1°C/1LSB
    // data_.voltage = (float)fb->voltage * 0.01f;
    // current in A, 0.01A/LSB
    data_.curr = regInfo_.isReverse ? -static_cast<float>(fb->current) * 0.01f :
                                      static_cast<float>(fb->current) * 0.01f;
    data_.torq = data_.curr * status_.torqueConstant;
    errorState_ = fb->errorState;

    return STM_OK;
}

MotorTypeDef_e LKMotorRS485::parseState2()
{
    if (checkSum(this->rxBuf_, 7))
        return STM_FAIL;

    State2_s *fb = (State2_s *)(this->rxBuf_ + 5);
    data_.tempture = fb->temperature;                     // 1°C/1LSB
    data_.curr = static_cast<float>(fb->current) * 0.01f; // current in A, 0.01A/LSB, for MF/MG motor
    data_.torq = data_.curr * status_.torqueConstant;
    data_.spdRadps = deg2rad(static_cast<float>(fb->speed)); // 反馈输出轴速度，1dps/LSB
    data_.spdRpm = radps2rpm(data_.spdRadps);
    // 双编码器反馈的是输出轴的编码器数据， 14bit encoder range: 0-16383, 15bit encoder range: 0-32767, 16bit encoder range: 0-65535
    float noumenaAng = static_cast<float>(fb->encoder) / this->span() * 2.f * PI;
    data_.rawAng = regInfo_.isReverse ? (2.f * PI) - noumenaAng : noumenaAng;
    data_.ang = data_.rawAng;

    /*
     * 在零度附近编码器会在0和6.28之间跳变，而多圈位置控制2是支持控制正负的，所以通过delta来判断编码器是否跨过零度，并计算圈数
     * 1. 当编码器从0跳变到6.28时，delta会大于PI，说明电机反向跨过零度，圈数减1，并且多圈角度等于编码器值减去2PI
     * 2. 当编码器从6.28跳变到0时，delta会小于-PI，说明电机正向跨过零度，圈数加1，并且多圈角度等于编码器值加上2PI
     * 3. 当编码器在零度附近跳变，多圈角通过抵消从而不会跳变
     */
    float delta = data_.ang - data_.angLast;
    if (delta > PI) {
        data_.cirNum--;
    } else if (delta < -PI) {
        data_.cirNum++;
    }
    data_.multipCirAng = data_.ang + (TWO_PI * data_.cirNum);

    /*
     * 电机内部的单圈认定范围是[-PI, PI], 如果超过范围，会将当前角度设置为0
     */
    data_.singleCirAng = rangeMap(data_.multipCirAng, -PI, PI);

    data_.angLast = data_.ang;

    return STM_OK;
}

MotorTypeDef_e LKMotorRS485::parseCtrlCmd()
{
    if (checkSum(this->rxBuf_, 7))
        return STM_FAIL;

    if (this->rxBuf_[5] == (uint8_t)ParamID_e::ANGLE_PID) {
        uint16_t kp, ki, kd;
        memcpy(&kp, this->rxBuf_ + 6, 2);
        memcpy(&ki, this->rxBuf_ + 8, 2);
        memcpy(&kd, this->rxBuf_ + 10, 2);
    } else if (this->rxBuf_[5] == (uint8_t)ParamID_e::SPEED_PID) {
        uint16_t kp, ki, kd;
        memcpy(&kp, this->rxBuf_ + 6, 2);
        memcpy(&ki, this->rxBuf_ + 8, 2);
        memcpy(&kd, this->rxBuf_ + 10, 2);
    } else if (this->rxBuf_[5] == (uint8_t)ParamID_e::CURRENT_PID) {
        uint16_t kp, ki, kd;
        memcpy(&kp, this->rxBuf_ + 6, 2);
        memcpy(&ki, this->rxBuf_ + 8, 2);
        memcpy(&kd, this->rxBuf_ + 10, 2);
    } else if (this->rxBuf_[5] == (uint8_t)ParamID_e::INPUT_TORQUE_LIMIT) {
        uint16_t torqueLimit;
        memcpy(&torqueLimit, this->rxBuf_ + 6, 2);
    } else if (this->rxBuf_[5] == (uint8_t)ParamID_e::INPUT_SPEED_LIMIT) {
        int32_t speedLimit;
        memcpy(&speedLimit, this->rxBuf_ + 6, 4);
    } else if (this->rxBuf_[5] == (uint8_t)ParamID_e::INPUT_ANGLE_LIMIT) {
        int32_t angleLimit;
        memcpy(&angleLimit, this->rxBuf_ + 6, 4);
    } else if (this->rxBuf_[5] == (uint8_t)ParamID_e::INPUT_CURRENT_RAMP) {
        int32_t currentRamp;
        memcpy(&currentRamp, this->rxBuf_ + 6, 4);
    } else if (this->rxBuf_[5] == (uint8_t)ParamID_e::INPUT_SPEED_RAMP) {
        int32_t speedRamp;
        memcpy(&speedRamp, this->rxBuf_ + 6, 4);
    } else {
        return STM_FAIL;
    }
    return STM_OK;
}

// 14bit encoder range: 0-16383, 15bit encoder range: 0-32767, 16bit encoder range: 0-65535
MotorTypeDef_e LKMotorRS485::parseEncoder()
{
    if (checkSum(this->rxBuf_, 6))
        return STM_FAIL;

    uint16_t encoder, encoderRaw, encoderOffset;
    memcpy(&encoder, this->rxBuf_ + 5, 2);
    memcpy(&encoderRaw, this->rxBuf_ + 7, 2);
    memcpy(&encoderOffset, this->rxBuf_ + 9, 2);

    return STM_OK;
}

MotorTypeDef_e LKMotorRS485::parsePosZero()
{
    if (checkSum(this->rxBuf_, 2))
        return STM_FAIL;

    uint16_t posZero;
    memcpy(&posZero, this->rxBuf_ + 5, 2);

    return STM_OK;
}

MotorTypeDef_e LKMotorRS485::parseMultiPos()
{
    if (checkSum(this->rxBuf_, 8))
        return STM_FAIL;

    int64_t pos;
    memcpy(&pos, this->rxBuf_ + 5, 8);

    this->data_.multipCirAng = deg2rad(static_cast<float>(pos) * 0.01f); // 0.01 degree/LSB
    this->data_.cirNum = this->data_.multipCirAng / (2.f * PI);

    return STM_OK;
}

MotorTypeDef_e LKMotorRS485::parseSinglePos()
{
    if (checkSum(this->rxBuf_, 4))
        return STM_FAIL;

    uint32_t pos;
    memcpy(&pos, this->rxBuf_ + 5, 4); // bug

    float noumenaAng = static_cast<float>(pos) / status_.innerReductionRatio; // range: 0~36000-1
    noumenaAng = deg2rad(noumenaAng * 0.01f);                                 // 0.01 degree/LSB
    noumenaAng = regInfo_.isReverse ? (2.f * PI) - noumenaAng : noumenaAng;
    this->data_.rawAng = noumenaAng;
    this->data_.ang = this->data_.rawAng;
    this->data_.angLast = this->data_.rawAng;
    this->data_.singleCirAng = this->data_.rawAng;

    return STM_OK;
}

MotorTypeDef_e LKMotorRS485::parseSetPos()
{
    if (checkSum(this->rxBuf_, 4))
        return STM_FAIL;

    int32_t pos;
    memcpy(&pos, this->rxBuf_ + 5, 4);

    this->data_.multipCirAng = deg2rad(static_cast<float>(pos) * 0.01f); // 0.01 degree/LSB

    return STM_OK;
}

template <uint8_t len> MotorTypeDef_e LKMotorRS485::send(const TransmitMsg_s<len> *_txBuf, uint8_t _len)
{
    memcpy(txBuf_, _txBuf, _len);
    MotorTypeDef_e ret = (MotorTypeDef_e)uart_.transmitDma(txBuf_, _len);
    return ret;
}

MotorTypeDef_e LKMotorRS485::txConvert(const uint8_t _cmdid)
{
    TransmitMsg_s<0> data;
    data.CMD = _cmdid;
    data.ID = this->regInfo_.offsetId;
    data.CMD_SUM = 0;
    data.CMD_SUM += data.head;
    data.CMD_SUM += data.CMD;
    data.CMD_SUM += data.ID;
    return send(&data, sizeof(data));
}

template <uint8_t len>
requires(len > 0) MotorTypeDef_e LKMotorRS485::txConvert(const uint8_t _cmdid, const uint8_t *_data)
{
    TransmitMsg_s<len> data;
    data.CMD = _cmdid;
    data.ID = this->regInfo_.offsetId;
    data.CMD_SUM = 0;
    data.CMD_SUM += data.head;
    data.CMD_SUM += data.CMD;
    data.CMD_SUM += len;
    data.CMD_SUM += data.ID;
    data.DATA_SUM = 0;
    for (uint8_t i = 0; i < len; i++) {
        data.DATA[i] = _data[i];
        data.DATA_SUM += _data[i];
    }

    return send(&data, sizeof(data));
}

MotorTypeDef_e LKMotorRS485::readState1andError() { return txConvert(0x9A); }

MotorTypeDef_e LKMotorRS485::cleanError() { return txConvert(0x9B); }

MotorTypeDef_e LKMotorRS485::readState2() { return txConvert(0x9C); }

MotorTypeDef_e LKMotorRS485::readState3() { return txConvert(0x9D); }

MotorTypeDef_e LKMotorRS485::disable() { return txConvert(0x80); }

MotorTypeDef_e LKMotorRS485::enable() { return txConvert(0x88); }

MotorTypeDef_e LKMotorRS485::stop() { return txConvert(0x81); }

MotorTypeDef_e LKMotorRS485::openloopCtrl()
{
    const int16_t powerControl = std::clamp((int16_t)(this->cmd_.elec), (int16_t)-850, (int16_t)850);
    return txConvert<2>(0xA0, (uint8_t *)&powerControl);
}

MotorTypeDef_e LKMotorRS485::torqCtrl()
{
    const int16_t iqControl = std::clamp((int16_t)(this->cmd_.elec), (int16_t)-2048, (int16_t)2048);
    return txConvert<2>(0xA1, (uint8_t *)&iqControl);
}

MotorTypeDef_e LKMotorRS485::velCtrl()
{
    const float speed = std::clamp(this->cmd_.vel, (float)-this->status_.speedMax, (float)this->status_.speedMax);
    const int32_t speedControl = (int32_t)(rad2deg(speed) * 100); //0.01dps/LSB
    return txConvert<4>(0xA2, (uint8_t *)&speedControl);
}

MotorTypeDef_e LKMotorRS485::multiPosCtrl1()
{
    const int64_t angleControl = (int64_t)(rad2deg(this->cmd_.pos) * 100); // 0.01degree/LSB
    return txConvert<8>(0xA3, (uint8_t *)&angleControl);
}

/**
 * @brief 减速前控制
 * @note 断控前的位置必须保证在[-PI,PI],否则电机内部识别为过圈，并且将当前角度为0
 * @return MotorTypeDef_e
 */
MotorTypeDef_e LKMotorRS485::multiPosCtrl2()
{
    const int64_t angleControl = (int64_t)(rad2deg(this->cmd_.pos * this->status_.innerReductionRatio) *
                                           100); // 0.01degree/LSB, 控制为减速前
    float speed;
    if (this->cmd_.vel == 0) {
        speed = 0.01f; // if speed is zero, maxSpeed will be set MAX
    } else {
        speed = this->cmd_.vel;
    }
    const uint32_t maxSpeed = (uint32_t)(rad2deg(speed) * 100); // 0.01dps/LSB
    uint8_t data[12];
    memcpy(data, &angleControl, 8);
    memcpy(data + 8, &maxSpeed, 4);
    return txConvert<12>(0xA4, data);
}

MotorTypeDef_e LKMotorRS485::singlePosCtrl1()
{
    const uint8_t spinDirection = this->regInfo_.isReverse;
    const float angle = rangeMap(this->cmd_.pos);
    const uint16_t angleControl = (uint16_t)(rad2deg(angle) * 100); // range:0~35999, 0.01degree/LSB, 0~360 degree
    uint8_t data[4];
    memcpy(data, &spinDirection, 1);
    memcpy(data + 1, &angleControl, 2);
    data[3] = 0;
    return txConvert<4>(0xA5, (uint8_t *)&data);
}

MotorTypeDef_e LKMotorRS485::singlePosCtrl2()
{
    const uint8_t spinDirection = this->regInfo_.isReverse;
    const float angle = rangeMap(this->cmd_.pos);
    const uint16_t angleControl = (uint16_t)(rad2deg(angle) * 100);      // range:0~35999, 0.01degree/LSB, 0~360 degree
    const uint32_t maxSpeed = (uint32_t)(rad2deg(this->cmd_.vel) * 100); // 0.01dps/LSB
    uint8_t data[8];
    memcpy(data, &spinDirection, 1);
    memcpy(data + 1, &angleControl, 2);
    data[3] = 0;
    memcpy(data + 4, &maxSpeed, 4);
    return txConvert<8>(0xA5, (uint8_t *)&data);
}

MotorTypeDef_e LKMotorRS485::incrementalPosCtrl1()
{
    const int32_t angleControl =
            (int32_t)(rad2deg(regInfo_.isReverse ? -this->cmd_.pos : this->cmd_.pos) * 100); // 0.01degree/LSB
    uint8_t data[4];
    memcpy(data, &angleControl, 4);
    return txConvert<4>(0xA5, (uint8_t *)&data);
}

MotorTypeDef_e LKMotorRS485::incrementalPosCtrl2()
{
    const int32_t angleControl =
            (int32_t)(rad2deg(regInfo_.isReverse ? -this->cmd_.pos : this->cmd_.pos) * 100); // 0.01degree/LSB
    const uint32_t maxSpeed = (uint32_t)(rad2deg(this->cmd_.vel) * 100);                     // 0.01dps/LSB
    uint8_t data[8];
    memcpy(data, &angleControl, 4);
    memcpy(data + 4, &maxSpeed, 4);
    return txConvert<88>(0xA5, (uint8_t *)&data);
}

MotorTypeDef_e LKMotorRS485::readCtrlCmd(const ParamID_e _id)
{
    uint8_t data[7]{};
    data[0] = static_cast<uint8_t>(_id);
    return txConvert<7>(0xC0, data);
}

MotorTypeDef_e LKMotorRS485::writeCtrlCmd(const ParamID_e _id, const std::array<uint8_t, 6> _data)
{
    uint8_t data[7]{};
    data[0] = static_cast<uint8_t>(_id);
    memcpy(data + 1, _data.data(), 6);
    return txConvert<7>(0xC1, data);
}

MotorTypeDef_e LKMotorRS485::readEncoder() { return txConvert(0x90); }

MotorTypeDef_e LKMotorRS485::setZeroPos() { return txConvert(0x19); }

MotorTypeDef_e LKMotorRS485::readMultiPos() { return txConvert(0x92); }

MotorTypeDef_e LKMotorRS485::clearPosCircle() { return txConvert(0x93); }

MotorTypeDef_e LKMotorRS485::readSinglePos() { return txConvert(0x94); }

MotorTypeDef_e LKMotorRS485::setPos(const float _angle)
{
    const int32_t angleControl = (int32_t)(rad2deg(_angle) * 100); // 0.01degree/LSB
    return txConvert<4>(0x95, (uint8_t *)&angleControl);
}

MotorTypeDef_e LKMotorRS485::ctrl()
{
    MotorTypeDef_e rslt = 0;
    if (!this->cmd_.SW && this->cmd_.prevSW) {
        rslt |= disable();
    } else if (!this->cmd_.SW && !this->cmd_.prevSW) {
        if (checkSend())
            rslt |= readState2();
    } else if (this->cmd_.SW && !this->cmd_.prevSW) {
        rslt |= enable();
    } else if (this->cmd_.SW) {
        if (checkSend())
            rslt |= (this->*ctrlCallback)();
    }
    return rslt;
}

MotorTypeDef_e LKMotorRS485::update()
{
    if (xQueueReceive(AUX_.rxQueue, this->rxBuf_, 0) == pdTRUE) {
        AUX_.recvCnt++;
        parse();
    }
    calcRecvFreq();

    taskENTER_CRITICAL();
    this->parseCmd();
    taskEXIT_CRITICAL();

    return ctrl();
}
