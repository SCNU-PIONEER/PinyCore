#include "UTMotor.hpp"
#include "Bsp_uart.hpp"
#include "Bsp_dma.hpp"
#include "Soc.hpp"
#include "Crc.hpp"

#include "../../../Utils/MotorCommonMacros.hpp"

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
                 DMA_HandleTypeDef *_dmaHandle)
        : IMotor(_name, std::move(_config))
{
    dmaHandle_ = _dmaHandle;
    txBuf_ = (uint8_t *)Dma::instance().ram_alloc(sizeof(TransmitMsg_s));
    rxBuf_ = (uint8_t *)Dma::instance().ram_alloc(sizeof(Feedback_s));

    this->cmd_.clear();
}

UTMotor::~UTMotor() { this->cancelMotor(); }

void UTMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

bool UTMotor::isEnable() const { return this->cmd_.SW; }

uint16_t UTMotor::getSendId() const { return this->model_.txBaseId; }

uint16_t UTMotor::getReceiveId() const { return this->model_.rxBaseId; }

uint16_t UTMotor::uid() { return getReceiveId(); }

void UTMotor::registerRecvCallback()
{
    Uart::instance().registerCallback(
            reinterpret_cast<UART_HandleTypeDef *>(this->pComHandle_),
            [this](UART_HandleTypeDef *_huart, uint16_t _dataLength) {
                // basic cb
                this->parse(
                        reinterpret_cast<const uint8_t *>(_huart->pRxBuffPtr));
                // user cb
                if (this->userRecvCallback_ != nullptr) {
                    this->userRecvCallback_(reinterpret_cast<const uint8_t *>(
                            _huart->pRxBuffPtr));
                }
            });
}

MotorTypeDef_e UTMotor::send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len)
{
    SET_485_1_DE_UP();
    memcpy(txBuf_, _txBuf, _len);
    MotorTypeDef_e ret = (MotorTypeDef_e)HAL_UART_Transmit_DMA(
            reinterpret_cast<UART_HandleTypeDef *>(this->pComHandle_),
            (uint8_t *)&txBuf_, _len);
    SET_485_1_DE_DOWN();
    return ret;
}

MotorTypeDef_e UTMotor::parse(const uint8_t *_rxBuf)
{
    Feedback_s *fb =
            reinterpret_cast<Feedback_s *>(const_cast<uint8_t *>(_rxBuf));
    if (fb->CRC16 != Get_CRC16_Check_Sum((uint8_t *)(fb), 14, 0)) {
        return 0; //TODO: CRC error
    } else {
        this->status_.error_ = static_cast<ErrorStatus_e>(fb->mode.status);
        this->data_.lastRawScale = this->data_.rawScale;
        this->data_.rawScale = fb->fbk.pos;
        this->data_.multipCirAng += 6.2832f *
                                    ((float)this->data_.rawScale -
                                     (float)this->data_.lastRawScale) /
                                    32768 / this->RR();
        this->data_.singleCirAng =
                rangeMap(this->data_.singleCirAng, 0, 2 * PI);
        this->data_.spdRadps = ((float)fb->fbk.speed / 256) * 6.2832f;
        this->data_.spdRpm = radps2rpm(this->data_.spdRadps);
        this->data_.torq = ((float)fb->fbk.torque) / 256;
        this->data_.curr = this->data_.torq / status_.Kn;
        this->data_.tempture = fb->fbk.temp;
    }

    (MotorTypeDef_e) HAL_UARTEx_ReceiveToIdle_DMA(
            reinterpret_cast<UART_HandleTypeDef *>(this->pComHandle_),
            (uint8_t *)&txBuf_, sizeof(TransmitMsg_s));
    __HAL_DMA_DISABLE_IT(dmaHandle_, DMA_IT_HT);
    return 0; //TODO: return check
}

void UTMotor::convert(TransmitMsg_s &_txBuf, const Cmd_s &_cmd)
{
    float p_des = cmd_.pos * this->RR();
    float v_des = cmd_.vel * this->RR();
    float t_ff = cmd_.torq * this->RR();
    clamp(t_ff, -127.99f, 127.99f);
    clamp(v_des, -804.00f, 804.00f);
    clamp(p_des, -411774.0f, 411774.0f);

    _txBuf.head[0] = 0xFE;
    _txBuf.head[1] = 0xEE;
    _txBuf.mode.status = cmd_.SW;
    _txBuf.mode.id = ctrlId_;
    _txBuf.comd.k_pos = Kp_ / 25.6f * 32768;
    _txBuf.comd.k_spd = Kd_ / 25.6f * 32768;
    _txBuf.comd.pos_des = p_des / 6.2832f * 32768;
    _txBuf.comd.spd_des = v_des / 6.2832f * 256;
    _txBuf.comd.tor_des = t_ff * 256;
    _txBuf.CRC16 =
            Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&_txBuf), 15, 0);
    this->cmd_.elec = this->data_.torq /
                      status_.Kn; // MIT unsupport return expected current
}


MotorTypeDef_e UTMotor::ctrl()
{
    TransmitMsg_s _txBuf;
    convert(_txBuf, this->cmd_);
    return send(this->model_.txBaseId, (uint8_t *)&_txBuf,
                sizeof(TransmitMsg_s));
}

MotorTypeDef_e UTMotor::update()
{
    if (xQueueReceive(this->rxQueue_, this->rxBuf_, 0) == pdTRUE) {
        this->parse(this->rxBuf_);
        this->calcRecvFreq();
        if (this->userRecvCallback_ != nullptr) {
            this->userRecvCallback_(this->rxBuf_);
        }
    }
    if (xQueueReceive(this->cmdQueue_, &this->cmdBuf_, 0) == pdTRUE) {
        this->parseCmd();
    }
    MotorTypeDef_e rslt = ctrl();
    return rslt;
}

void UTMotor::setKp(float Kp_) { clamp(Kp_, 0.0f, 25.599f); }

void UTMotor::setKd(float Kd_) { clamp(Kd_, 0.0f, 25.599f); }

MotorTypeDef_e UTMotor::enable()
{
    TransmitMsg_s _txBuf;
    this->cmd_.updateSW(true);
    convert(_txBuf, this->cmd_);
    return send(ctrlId_, (uint8_t *)&_txBuf, sizeof(TransmitMsg_s));
}

MotorTypeDef_e UTMotor::disable()
{
    TransmitMsg_s _txBuf;
    this->cmd_.updateSW(false);
    convert(_txBuf, this->cmd_);
    return send(ctrlId_, (uint8_t *)&txBuf_, sizeof(TransmitMsg_s));
}
