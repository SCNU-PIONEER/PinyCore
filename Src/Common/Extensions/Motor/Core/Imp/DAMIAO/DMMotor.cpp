#include "DMMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

#include "Bsp_can.hpp"

using namespace PINYMOTOR;
using namespace DMMOTOR;

Status_s &Status_s::operator=(const Status_s &_other)
{
    if (this != &_other) {
        PMax = _other.PMax;
        VMax = _other.VMax;
        TMax = _other.TMax;
        MITKpMax = _other.MITKpMax;
        MITKdMax = _other.MITKdMax;
        currTxCodeSpan = _other.currTxCodeSpan;
        currRated = _other.currRated;
        torqRated = _other.torqRated;
        currMax = _other.currMax;
        torqMax = _other.torqMax;
        torqConstant = _other.torqConstant;
    }
    return *this;
}

DMMotor::DMMotor(const char _name[16], InitConfig_s _config)
        : Base(_name, std::move(_config))

{
    this->rxQueue_ = xQueueCreate(3, sizeof(RxBus_s::CANRxBuf_s<8>::data));
}

DMMotor::~DMMotor()
{
    this->cancelRecvCallback();
    this->cancelMotor();
    LOG::info("DMMotor",
              " %s: An instance of DMMotor created, rxBaseId:%hx, txBaseId:%hx",
              this->name_, this->model_.rxBaseId, this->model_.txBaseId);
}

void DMMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

bool DMMotor::isEnable() const { return this->cmd_.SW; }

uint16_t DMMotor::canId() const
{
    return this->model_.txBaseId + this->offsetId_;
}

uint16_t DMMotor::masterId() const
{
    return this->model_.rxBaseId + this->offsetId_;
}

uint16_t DMMotor::uid() { return masterId(); }

void DMMotor::registerRecvCallback()
{
    // lamda
    Can::instance().registerCallback(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->masterId(),
            [this](const uint8_t *_rxBuf) {
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(this->rxQueue_, _rxBuf,
                                  &higherPriorityTaskWoken);
            });
    LOG::info("DMMotor", " %s: Receive cb registed, masterId:%hx", this->name_,
              this->masterId());
}


void DMMotor::cancelRecvCallback()
{
    Can::instance().unregisterCallback(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->masterId());
    LOG::info("DMMotor", " %s: Receive cb canceled, masterId:%hx", this->name_,
              this->masterId());
}

void DMMotor::updateCtrlId()
{
    switch (this->workMode_) {
    case WorkMode_e::MIT_TT:
    case WorkMode_e::MIT_VDESPDES:
    case WorkMode_e::MIT_VDES: {
        this->ctrlId_ = this->canId();
        break;
    }
    case WorkMode_e::PDESVDES: {
        this->ctrlId_ = this->canId() + 0x100;
        break;
    }
    case WorkMode_e::VDES: {
        this->ctrlId_ = this->canId() + 0x200;
        break;
    }
    case WorkMode_e::EMIT: {
        this->ctrlId_ = this->canId() + 0x300;
        break;
    }
    default: {
        LOG::error("DMMotor", " %s: this mode is not supported", this->name_);
        this->ctrlId_ = 0xFFFF;
        break;
    }
    }
}

void DMMotor::setMITKp(float _kp)
{
    if (_kp < 0 || _kp > status_.MITKpMax) {
        LOG::error("DMMotor", " %s: MITKp out of range", this->name_);
        return;
    }
    MITKp_ = _kp;
}

void DMMotor::setMITKd(float _kd)
{
    if (_kd < 0 || _kd > status_.MITKdMax) {
        LOG::error("DMMotor", " %s: MITKd out of range", this->name_);
        return;
    }
    MITKd_ = _kd;
}

MotorTypeDef_e DMMotor::send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len)
{
    if (this->checkSend()) {
        // Check this Buffer
        // LOG::warn("DMMotor", " %s: send data to CAN %hx", this->name_, _sendId);
        // LOG::warn("DMMotor",
        //           " %s: txBuf: %02X %02X %02X %02X %02X %02X %02X %02X",
        //           this->name_, _txBuf[0], _txBuf[1], _txBuf[2], _txBuf[3],
        //           _txBuf[4], _txBuf[5], _txBuf[6], _txBuf[7]);

        return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                reinterpret_cast<canHandle *>(this->pComHandle_), _sendId,
                _txBuf, _len));
    } else {
        return 0;
    }
}

MotorTypeDef_e DMMotor::parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf)
{
    // 先处理非常规数据反馈的帧
    if (_rxBuf.data[0] == static_cast<uint8_t>(canId()) &&
        _rxBuf.data[1] == static_cast<uint8_t>(canId() >> 8)) {
        uint32_t rawDat = (_rxBuf.data[7] << 24) | (_rxBuf.data[6] << 16) |
                          (_rxBuf.data[5] << 8) | _rxBuf.data[4];
        auto it = regObjList_.find(static_cast<RegId_e>(_rxBuf.data[3]));
        if (_rxBuf.data[2] == 0x33) {
            // 读反馈
            (*it).second->isRead = true;
            memcpy(&(*it).second->dat, &rawDat, 4);
        } else if (_rxBuf.data[2] == 0x55) {
            // 写反馈
            (*it).second->isWrite = true;
        } else if (_rxBuf.data[2] == 0xAA) {
            // 存储反馈
            (*it).second->isStorage = true;
        } else {
            LOG::error("DMMotor", " %s: Unknown feedback type, rxBuf[2]:%02X",
                       this->name_, _rxBuf.data[2]);
            return 1;
        }
    } else {
        Feedback_s fb = {};
        fb.ID = _rxBuf.data[0] & 0x0F;
        fb.errorCode = static_cast<ErrorCode_e>(_rxBuf.data[0] >> 4);
        fb.rawAng = (_rxBuf.data[1] << 8) | _rxBuf.data[2];
        fb.rawVel = (_rxBuf.data[3] << 4) | (_rxBuf.data[4] >> 4);
        fb.torque = ((_rxBuf.data[4] & 0xF) << 8 | _rxBuf.data[5]);
        fb.mosTemperature = _rxBuf.data[6];
        fb.rotorTemperature = _rxBuf.data[7];

        errorCode_ = fb.errorCode;

        float noumenaAng =
                static_cast<float>(fb.rawAng) / this->span() * 2.f * PI;
        this->data_.rawAng = this->isReverse_ ? (2.f * PI) - noumenaAng :
                                                noumenaAng;
        float del = this->data_.rawAng - this->data_.zeroAng;
        this->data_.ang = del < 0 ? del + (2.f * PI) : del;

        float noumenaVel =
                uint2float(fb.rawVel, -status_.VMax, status_.VMax, 12) /
                this->rr();
        this->data_.spdRadps = this->isReverse_ ? -noumenaVel : noumenaVel;
        this->data_.spdRpm = radps2rpm(this->data_.spdRadps);

        float noumenaTorq =
                uint2float(fb.torque, -status_.TMax, status_.TMax, 12) *
                this->rr();
        this->data_.torq = this->isReverse_ ? -noumenaTorq : noumenaTorq;
        this->data_.curr = this->data_.torq / status_.torqConstant;

        this->data_.tempture = fb.mosTemperature;

        float angDiff =
                (getMinorArc(this->data_.ang, this->data_.angLast, 2.f * PI)) /
                this->rr();

        if ((this->globalState_ == GlobalState_e::OFFLINE ||
             this->globalState_ == GlobalState_e::UNREGISTER) &&
            this->data_.angLast != this->data_.rawAng) {
            this->globalState_ = GlobalState_e::ONLINE;
            angDiff = 0;
            this->data_.multipCirAng =
                    this->data_.rawAng / this->rr(); // 与电机内编码器同步零点
        }

        this->data_.angLast = this->data_.rawAng;

        this->data_.multipCirAng += angDiff;
        this->data_.cirNum = this->data_.multipCirAng / (2.f * PI);

        if (this->rr() == 1)
            this->data_.singleCirAng = this->data_.ang;
        else
            this->data_.singleCirAng = rangeMap(this->data_.multipCirAng);
    }
    return 0;
}

MotorTypeDef_e DMMotor::ctrl()
{
    MotorTypeDef_e rslt = 0;
    typedef union {
        MITMsg_s msgMIT;
        PDESVDESMsg_s msgPDESVDES;
        VDESMsg_s msgVDES;
        EMITMsg_s msgEMIT;
    } DMMsg_u;
    DMMsg_u dmMsg = {};
    uint8_t txBuf[8] = {};
    uint8_t lenBuf = 0;
    bool isMIT = false;
    switch (this->workMode_) {
    case WorkMode_e::QUAD_CURR: {
        LOG::error("DMMotor", " %s: QUAD_CURR mode is not supported",
                   this->name_);
        break;
    }
    case WorkMode_e::QUAD_VOLT: {
        LOG::error("DMMotor", " %s: QUAD_VOLT mode is not supported",
                   this->name_);
        break;
    }
    case WorkMode_e::MIT_TT: {
        dmMsg.msgMIT.Kp = 0;
        dmMsg.msgMIT.Kd = 0;
        isMIT = true;
        if (this->cmd_.curCmdType == MotorCmdType_e::SET_VEL) {
            this->cmd_.torq =
                    this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        } else if (this->cmd_.curCmdType == MotorCmdType_e::SET_POS) {
            this->cmd_.vel = this->posPID_->calc(
                    getMinorArc(this->cmd_.pos, this->data_.singleCirAng,
                                2.f * PI),
                    0);
            if (!(this->cmd_.velMax < 0.f)) {
                this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                            this->cmd_.velMax);
            }
            this->cmd_.torq =
                    this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        }
        this->cmd_.torq = this->isReverse_ ? -this->cmd_.torq : this->cmd_.torq;
        dmMsg.msgMIT.torqueForward =
                float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

        this->cmd_.elec =
                this->cmd_.torq /
                status_.torqConstant; // MIT_TT support return expected current
        this->cmd_.elec = this->isReverse_ ? -this->cmd_.elec : this->cmd_.elec;
        break;
    }
    case WorkMode_e::MIT_VDES: {
        dmMsg.msgMIT.Kd = float2uint(this->MITKd_, -status_.MITKdMax,
                                     status_.MITKdMax, 12);
        dmMsg.msgMIT.Kp = 0;
        isMIT = true;
        if (this->cmd_.curCmdType == MotorCmdType_e::SET_POS) {
            this->cmd_.vel = this->posPID_->calc(
                    getMinorArc(this->cmd_.pos, this->data_.singleCirAng,
                                2.f * PI),
                    0);
        }
        this->cmd_.vel = this->isReverse_ ? -this->cmd_.vel : this->cmd_.vel;
        dmMsg.msgMIT.exptVel =
                float2uint(this->cmd_.vel, -status_.VMax, status_.VMax, 12);
        // forward torque
        this->cmd_.torq = this->isReverse_ ? -this->cmd_.torq : this->cmd_.torq;
        dmMsg.msgMIT.torqueForward =
                float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

        this->cmd_.elec =
                this->data_.torq /
                status_.torqConstant; // MIT_VDES unsupport return expected current
        break;
    }
    case WorkMode_e::MIT_VDESPDES: {
        this->cmd_.pos = this->isReverse_ ? -this->cmd_.pos : this->cmd_.pos;
        this->cmd_.vel = this->isReverse_ ? -this->cmd_.vel : this->cmd_.vel;
        dmMsg.msgMIT.exptScale =
                float2uint(this->cmd_.pos, -status_.PMax, status_.PMax, 16);
        dmMsg.msgMIT.exptVel =
                float2uint(this->cmd_.vel, -status_.VMax, status_.VMax, 12);
        dmMsg.msgMIT.Kd = float2uint(this->MITKd_, -status_.MITKdMax,
                                     status_.MITKdMax, 12);
        dmMsg.msgMIT.Kp = float2uint(this->MITKp_, -status_.MITKpMax,
                                     status_.MITKpMax, 12);
        isMIT = true;
        // forward torque
        this->cmd_.torq = this->isReverse_ ? -this->cmd_.torq : this->cmd_.torq;
        dmMsg.msgMIT.torqueForward =
                float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

        this->cmd_.elec =
                this->data_.torq /
                status_.torqConstant; // MIT_VDESPDES unsupport return expected current
        break;
    }
    case WorkMode_e::PDESVDES: {
        lenBuf = 8;
        this->cmd_.pos = this->isReverse_ ? -this->cmd_.pos : this->cmd_.pos;
        this->cmd_.vel = this->isReverse_ ? -this->cmd_.vel : this->cmd_.vel;
        dmMsg.msgPDESVDES.exptScale = this->cmd_.pos;
        dmMsg.msgPDESVDES.exptVel = this->cmd_.vel;
        memcpy(txBuf, &dmMsg.msgPDESVDES.exptScale, 4);
        memcpy(&txBuf[4], &dmMsg.msgPDESVDES.exptVel, 4);

        this->cmd_.elec =
                this->data_.torq /
                status_.torqConstant; // PDESVDES unsupport return expected current
        break;
    }
    case WorkMode_e::VDES: {
        lenBuf = 4;
        if (this->cmd_.curCmdType == MotorCmdType_e::SET_POS) {
            this->cmd_.vel = this->posPID_->calc(
                    getMinorArc(this->cmd_.pos, this->data_.singleCirAng,
                                2.f * PI),
                    0);
        }
        this->cmd_.vel = this->isReverse_ ? -this->cmd_.vel : this->cmd_.vel;
        dmMsg.msgVDES.exptVel = this->cmd_.vel;
        memcpy(txBuf, &dmMsg.msgVDES.exptVel, 4);
        this->cmd_.elec =
                this->data_.torq /
                status_.torqConstant; // VDES unsupport return expected current
        break;
    }
    case WorkMode_e::EMIT: {
        lenBuf = 8;
        this->cmd_.pos = this->isReverse_ ? -this->cmd_.pos : this->cmd_.pos;
        this->cmd_.vel = this->isReverse_ ? -this->cmd_.vel : this->cmd_.vel;
        this->cmd_.torq = this->isReverse_ ? -this->cmd_.torq : this->cmd_.torq;
        dmMsg.msgEMIT.exptScale = this->cmd_.pos;
        dmMsg.msgEMIT.exptVelX100 = static_cast<uint16_t>(
                ((this->cmd_.vel < 0) ? -this->cmd_.vel : this->cmd_.vel) *
                100.f);
        dmMsg.msgEMIT.imaxX10000 = static_cast<uint16_t>(
                ((this->cmd_.torq < 0) ? -this->cmd_.torq : this->cmd_.torq) /
                status_.torqConstant / status_.currMax *
                status_.currTxCodeSpan);
        float f = dmMsg.msgEMIT.exptScale;
        memcpy(txBuf, &f, 4);
        txBuf[4] = static_cast<uint8_t>((dmMsg.msgEMIT.exptVelX100) >> 8);
        txBuf[5] = static_cast<uint8_t>(dmMsg.msgEMIT.exptVelX100);
        txBuf[6] = static_cast<uint8_t>((dmMsg.msgEMIT.imaxX10000) >> 8);
        txBuf[7] = static_cast<uint8_t>(dmMsg.msgEMIT.imaxX10000);

        this->cmd_.elec =
                this->data_.torq /
                status_.torqConstant; // EMIT unsupport return expected current
        break;
    }
    default: {
        LOG::error("DMMotor", " %s: this mode is not supported", this->name_);
        break;
    }
    }
    if (isMIT) {
        lenBuf = 8;
        txBuf[0] = static_cast<uint8_t>((dmMsg.msgMIT.exptScale & 0xFF00) >> 8);
        txBuf[1] = static_cast<uint8_t>(dmMsg.msgMIT.exptScale & 0x00FF);
        txBuf[2] = static_cast<uint8_t>((dmMsg.msgMIT.exptVel & 0x0FF0) >> 4);
        txBuf[3] = static_cast<uint8_t>((dmMsg.msgMIT.exptVel & 0x000F) << 4 |
                                        ((dmMsg.msgMIT.Kp & 0x0FF0) >> 8));
        txBuf[4] = static_cast<uint8_t>(dmMsg.msgMIT.Kp & 0x000F);
        txBuf[5] = static_cast<uint8_t>((dmMsg.msgMIT.Kd & 0x0FF0) >> 4);
        txBuf[6] = static_cast<uint8_t>(
                (dmMsg.msgMIT.Kd & 0x000F) << 4 |
                ((dmMsg.msgMIT.torqueForward & 0x0F00) >> 8));
        txBuf[7] = static_cast<uint8_t>(dmMsg.msgMIT.torqueForward & 0x00FF);
    }

    if ((this->cmd_.SW && !this->cmd_.prevSW) ||
        (this->cmd_.SW && errorCode_ == ErrorCode_e::MOTOR_DISABLE)) {
        this->enable();
    } else if (!this->cmd_.SW) {
        if (this->posPID_ != nullptr)
            this->posPID_->reset();
        if (this->velPID_ != nullptr)
            this->velPID_->reset();
        if (this->torqPID_ != nullptr)
            this->torqPID_->reset();
        this->disable();
    } else {
        rslt |= this->send(this->ctrlId_, txBuf_.data, txBuf_.len);
    }
    return rslt;
}

MotorTypeDef_e DMMotor::update()
{
    if (xQueueReceive(this->rxQueue_, this->rxBuf_.data, 0) == pdTRUE) {
        this->parse(this->rxBuf_);
        this->calcRecvFreq();
    }
    if (xQueueReceive(this->cmdQueue_, &this->cmdBuf_, 0) == pdTRUE) {
        this->parseCmd();
    }
    MotorTypeDef_e rslt = ctrl();
    return rslt;
}

MotorTypeDef_e DMMotor::enable()
{
    MotorTypeDef_e rslt = 0;
    // 定义一个8字节的数组enableCmdPack，用于存储使能命令
    uint8_t enableCmdPack[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC
    };
    rslt |= this->send(this->ctrlId_, enableCmdPack, 8);
    this->cmd_.updateSW(true); // force enable
    return rslt;
}

MotorTypeDef_e DMMotor::disable()
{
    MotorTypeDef_e rslt = 0;
    // 定义一个8字节的数组disableCmdPack，用于存储禁用命令
    uint8_t disableCmdPack[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD
    };
    rslt |= this->send(this->ctrlId_, disableCmdPack, 8);
    this->cmd_.updateSW(false); // force disable
    return rslt;
}

MotorTypeDef_e DMMotor::clearError()
{
    MotorTypeDef_e rslt = 0;
    uint8_t enableCmdPack[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB
    };
    rslt |= this->send(this->ctrlId_, enableCmdPack, 8);
    return rslt;
}
