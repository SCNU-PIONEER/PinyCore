#include "Crc.hpp"
#include "Bsp.hpp"
#include "UT8010_6.hpp"
#include "MotorCommonMacros.hpp"
#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace UTMOTOR;

Status_s &Status_s::operator=(const Status_s &_other)
{
    if (this != &_other) {
        PMax = _other.PMax;
        VMax = _other.VMax;
        TMax = _other.TMax;
        KpMax = _other.KpMax;
        KdMax = _other.KdMax;
        currMax = _other.currMax;
        torqMax = _other.torqMax;
        speedMax = _other.speedMax;
        Kn = _other.Kn;
    }
    return *this;
}

UTMotor::UTMotor(const char _name[16], InitConfig_s _config,
                 UART_HandleTypeDef *_huart, DMA_HandleTypeDef *_dmaHandle)
        : IMotor(_name, _config)
        , uart_(_huart, _dmaHandle)
        , txBuf_((TransmitMsg_s *)Dma::instance().ram_alloc(
                  sizeof(TransmitMsg_s)))
        , rxBuf_((Feedback_s *)Dma::instance().ram_alloc(sizeof(Feedback_s)))

{
    this->cmd_.clear();

    AUX_.rxQueue = xQueueCreate(4, sizeof(Feedback_s));

    /* send first frame to init dma reception */
    __HAL_UART_ENABLE_IT(
            reinterpret_cast<UART_HandleTypeDef *>(regInfo_.pComHandle),
            UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(
            reinterpret_cast<UART_HandleTypeDef *>(regInfo_.pComHandle),
            (uint8_t *)rxBuf_, sizeof(Feedback_s));
    __HAL_DMA_DISABLE_IT(uart_.hdma_, DMA_IT_HT);
}

UTMotor::~UTMotor() { this->cancelMotor(); }

void UTMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

bool UTMotor::isEnable() const { return this->cmd_.SW; }

uint16_t UTMotor::getSendId() const { return regInfo_.model.txBaseId; }

uint16_t UTMotor::getReceiveId() const { return regInfo_.model.rxBaseId; }

void UTMotor::registerRecvCallback()
{
    uart_.registerCallback(
            [this](UART_HandleTypeDef *_huart, uint16_t _dataLength) {
                // basic cb
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(AUX_.rxQueue, _huart->pRxBuffPtr,
                                  &higherPriorityTaskWoken);
            });
}

MotorTypeDef_e UTMotor::send(uint16_t _sendId, TransmitMsg_s *_txBuf,
                             uint8_t _len)
{
    SET_485_1_DE_UP();
    memcpy(txBuf_, _txBuf, _len);
    MotorTypeDef_e ret = (MotorTypeDef_e)HAL_UART_Transmit_DMA(
            reinterpret_cast<UART_HandleTypeDef *>(regInfo_.pComHandle),
            (uint8_t *)txBuf_, _len);
    SET_485_1_DE_DOWN();
    return ret;
}

MotorTypeDef_e UTMotor::parse(Feedback_s *_rxBuf)
{
    Feedback_s *fb = _rxBuf;
    if (fb->CRC16 != Get_CRC16_Check_Sum((uint8_t *)(fb), 14, 0)) {
        return 0; //TODO: CRC error
    } else {
        constexpr float B2C = 2 * PI / 32768;
        this->status_.error_ = static_cast<ErrorStatus_e>(fb->mode.status);
        this->data_.angLast = this->data_.rawAng;
        float noumenaAng = static_cast<float>(fb->fbk.pos) * B2C;
        this->data_.rawAng = regInfo_.isReverse ? (2 * PI) - noumenaAng :
                                                  noumenaAng;
        float del = this->data_.rawAng - this->data_.zeroAng;
        this->data_.ang = del < 0 ? del + (2 * std::numbers::pi_v<float>) : del;
        this->data_.multipCirAng +=
                (this->data_.rawAng - this->data_.angLast) / this->rr();
        this->data_.cirNum = this->data_.multipCirAng / (2 * PI);
        this->data_.singleCirAng =
                rangeMap(this->data_.multipCirAng, 0, (2 * PI));
        float noumenaVel = ((float)fb->fbk.speed / 256) * (2 * PI);
        this->data_.spdRadps = regInfo_.isReverse ? -noumenaVel : noumenaVel;
        this->data_.spdRpm = radps2rpm(this->data_.spdRadps);
        float noumenaTorq = ((float)fb->fbk.torque) / 256;
        this->data_.torq = regInfo_.isReverse ? -noumenaTorq : noumenaTorq;
        this->data_.curr = this->data_.torq / status_.Kn;
        this->data_.tempture = fb->fbk.temp;
    }

    (MotorTypeDef_e) HAL_UARTEx_ReceiveToIdle_DMA(
            reinterpret_cast<UART_HandleTypeDef *>(regInfo_.pComHandle),
            (uint8_t *)txBuf_, sizeof(TransmitMsg_s));
    __HAL_DMA_DISABLE_IT(uart_.hdma_, DMA_IT_HT);
    return 0; //TODO: return check
}

void UTMotor::convert(TransmitMsg_s &_txBuf, const Cmd_s &_cmd)
{
    float multiplier = regInfo_.isReverse ? -1 : 1;
    float pDes = cmd_.pos * this->rr() * multiplier;
    float vDes = cmd_.vel * this->rr() * multiplier;
    float tFF = cmd_.torq * this->rr() * multiplier;
    clamp(tFF, -127.99f, 127.99f);
    clamp(vDes, -804.00f, 804.00f);
    clamp(pDes, -411774.0f, 411774.0f);

    _txBuf.head[0] = 0xFE;
    _txBuf.head[1] = 0xEE;
    _txBuf.mode.status = cmd_.SW;
    _txBuf.mode.id = ctrlId_;
    _txBuf.comd.k_pos = static_cast<int16_t>(kp_ / 25.6f * 32768);
    _txBuf.comd.k_spd = static_cast<int16_t>(kd_ / 25.6f * 32768);
    _txBuf.comd.pos_des = static_cast<int32_t>(pDes / 6.2832f * 32768);
    _txBuf.comd.spd_des = static_cast<int16_t>(vDes / 6.2832f * 256);
    _txBuf.comd.tor_des = static_cast<int16_t>(tFF * 256);
    _txBuf.CRC16 =
            Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&_txBuf), 15, 0);
    this->cmd_.elec = this->data_.torq / status_.Kn;
}


MotorTypeDef_e UTMotor::ctrl()
{
    TransmitMsg_s txBuf{};
    convert(txBuf, this->cmd_);
    return send(regInfo_.model.txBaseId, &txBuf, sizeof(TransmitMsg_s));
}

void UTMotor::overrideReductionRatio(float _newReductionRatio)
{
    regInfo_.model.reductionRatio = _newReductionRatio;
    status_.torqMax *= _newReductionRatio;
    status_.Kn *= _newReductionRatio;
    LOG::info("UTMotor", " %s: you have changed reduction ratio to %f",
              regInfo_.name, _newReductionRatio);
}

MotorTypeDef_e UTMotor::update()
{
    if (xQueueReceive(AUX_.rxQueue, this->rxBuf_, 0) == pdTRUE) {
        AUX_.recvCnt++;
        this->parse(this->rxBuf_);
    }

    taskENTER_CRITICAL();
    this->parseCmd();
    taskEXIT_CRITICAL();

    this->calcRecvFreq();
    MotorTypeDef_e rslt = ctrl();
    return rslt;
}

void UTMotor::setKp(const float _kp) { kp_ = clamp(_kp, 0.0f, 25.599f); }

void UTMotor::setKd(const float _kd) { kd_ = clamp(_kd, 0.0f, 25.599f); }

MotorTypeDef_e UTMotor::enable()
{
    TransmitMsg_s txBuf{};
    this->cmd_.updateSW(true);
    convert(txBuf, this->cmd_);
    return send(ctrlId_, &txBuf, sizeof(TransmitMsg_s));
}

MotorTypeDef_e UTMotor::disable()
{
    TransmitMsg_s txBuf{};
    this->cmd_.updateSW(false);
    convert(txBuf, this->cmd_);
    return send(ctrlId_, &txBuf, sizeof(TransmitMsg_s));
}
