#include "DMMotor.hpp"

#include "../../../Utils/MotorCommonMacros.hpp"

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
    this->rxQueue_ = xQueueCreate(10, sizeof(RxBus_s::CANRxBuf_s));
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
                xQueueSendFromISR(this->rxQueue_, const_cast<uint8_t *>(_rxBuf),
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
    case WorkMode_e::QUAD_CURR: {
        LOG::error("DMMotor", " %s: QUAD_CURR mode is not supported",
                   this->name_);
        this->ctrlId_ = 0xFFFF;
        break;
    }
    case WorkMode_e::QUAD_VOLT: {
        LOG::error("DMMotor", " %s: QUAD_VOLT mode is not supported",
                   this->name_);
        this->ctrlId_ = 0xFFFF;
        break;
    }
    case WorkMode_e::MIT_TT: {
        this->ctrlId_ = this->canId();
        break;
    }
    case WorkMode_e::MIT_VDESPDES: {
        this->ctrlId_ = this->canId();
        break;
    }
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
#if 0
        // Check this Buffer
        log.debug(LOCATION, "DMMotor", " %s: send data to CAN %hx", this->name_,
                  _sendId);
        log.debug(LOCATION, "DMMotor",
                  " %s: txBuf: %02X %02X %02X %02X %02X %02X %02X %02X",
                  this->name_, _txBuf[0], _txBuf[1], _txBuf[2], _txBuf[3],
                  _txBuf[4], _txBuf[5], _txBuf[6], _txBuf[7]);
#endif
        this->lastSendTick = xTaskGetTickCount();
        return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                reinterpret_cast<canHandle *>(this->pComHandle_), _sendId,
                _txBuf, _len));
    } else {
        return 0;
    }
}

MotorTypeDef_e DMMotor::parse(const RxBus_s::CANRxBuf_s &_rxBuf)
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
        Feedback_s fb;
        fb.ID = _rxBuf.data[0] & 0x0F;
        fb.errorCode = static_cast<ErrorCode_e>(_rxBuf.data[0] >> 4);
        fb.rawScale = (_rxBuf.data[1] << 8) | _rxBuf.data[2];
        fb.rawVel = (_rxBuf.data[3] << 4) | (_rxBuf.data[4] >> 4);
        fb.torque = ((_rxBuf.data[4] & 0xF) << 8 | _rxBuf.data[5]);
        fb.mosTemperature = _rxBuf.data[6];
        fb.rotorTemperature = _rxBuf.data[7];

        errorCode_ = fb.errorCode;

        this->data_.rawScale = fb.rawScale;

        this->data_.spdRadps =
                uint2float(fb.rawVel, -status_.VMax, status_.VMax, 12) /
                this->RR();
        this->data_.spdRpm = radps2rpm(this->data_.spdRadps);
        this->data_.torq =
                uint2float(fb.torque, -status_.TMax, status_.TMax, 12) *
                this->RR();

        this->data_.curr = this->data_.torq / status_.torqConstant;

        this->data_.tempture = fb.mosTemperature;

        float angDiff = getMinorArc(this->data_.rawScale,
                                    this->data_.lastRawScale, this->span()) *
                        2 * PI / (this->span() * this->RR());
        if (this->globalState_ == GlobalState_e::OFFLINE &&
            this->data_.lastRawScale != this->data_.rawScale) {
            this->globalState_ = GlobalState_e::ONLINE;
            angDiff = 0;
        }
        this->data_.lastRawScale = this->data_.rawScale;

        this->data_.multipCirAng += angDiff;
        this->data_.singleCirAng += angDiff;
        this->data_.singleCirAng =
                rangeMap(this->data_.singleCirAng, 0, 2 * PI);
    }
    return 0;
}

MotorTypeDef_e DMMotor::ctrl()
{
    if (xQueueReceive(this->rxQueue_, this->rxBuf_.data, 0) == pdTRUE) {
        this->parse(this->rxBuf_);
        this->calcRecvFreq();
        if (this->userRecvCallback_ != nullptr) {
            this->userRecvCallback_(this->rxBuf_.data);
        }
    }
    MotorTypeDef_e rslt = 0;
    typedef union {
        MITMsg_s msgMIT;
        PDESVDESMsg_s msgPDESVDES;
        VDESMsg_s msgVDES;
        EMITMsg_s msgEMIT;
    } DMMsg_u;
    DMMsg_u DMMsg = {};
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
        DMMsg.msgMIT.Kp = 0;
        DMMsg.msgMIT.Kd = 0;
        isMIT = true;
        if (this->curCmdType_ == MotorCmdType_e::SET_VEL) {
            this->cmd_.torq =
                    this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        } else if (this->curCmdType_ == MotorCmdType_e::SET_POS) {
            this->cmd_.vel = this->posPID_->calc(
                    getMinorArc(this->cmd_.pos, this->data_.singleCirAng,
                                2.f * PI),
                    0);
            this->cmd_.torq =
                    this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        }
        DMMsg.msgMIT.torqueOffset =
                float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);
        break;
    }
    case WorkMode_e::MIT_VDES: {
        DMMsg.msgMIT.Kd = float2uint(this->MITKd_, -status_.MITKdMax,
                                     status_.MITKdMax, 12);
        DMMsg.msgMIT.Kp = 0;
        // forward torque
        DMMsg.msgMIT.torqueOffset =
                float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);
        isMIT = true;
        if (this->curCmdType_ == MotorCmdType_e::SET_POS) {
            this->cmd_.vel = this->posPID_->calc(
                    getMinorArc(this->cmd_.pos, this->data_.singleCirAng,
                                2.f * PI),
                    0);
        }
        DMMsg.msgMIT.exptVel =
                float2uint(this->cmd_.vel, -status_.VMax, status_.VMax, 12);
        break;
    }
    case WorkMode_e::MIT_VDESPDES: {
        DMMsg.msgMIT.exptScale =
                float2uint(this->cmd_.pos, -status_.PMax, status_.PMax, 16);
        DMMsg.msgMIT.exptVel =
                float2uint(this->cmd_.vel, -status_.VMax, status_.VMax, 12);
        DMMsg.msgMIT.Kd = float2uint(this->MITKd_, -status_.MITKdMax,
                                     status_.MITKdMax, 12);
        DMMsg.msgMIT.Kp = float2uint(this->MITKp_, -status_.MITKpMax,
                                     status_.MITKpMax, 12);
        // forward torque
        DMMsg.msgMIT.torqueOffset =
                float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);
        isMIT = true;
        break;
    }
    case WorkMode_e::PDESVDES: {
        lenBuf = 8;
        DMMsg.msgPDESVDES.exptScale = this->cmd_.pos;
        DMMsg.msgPDESVDES.exptVel = this->cmd_.vel;
        memcpy(txBuf, &DMMsg.msgPDESVDES.exptScale, 4);
        memcpy(&txBuf[4], &DMMsg.msgPDESVDES.exptVel, 4);
        break;
    }
    case WorkMode_e::VDES: {
        lenBuf = 4;
        if (this->curCmdType_ == MotorCmdType_e::SET_POS) {
            this->cmd_.vel = this->posPID_->calc(
                    getMinorArc(this->cmd_.pos, this->data_.singleCirAng,
                                2.f * PI),
                    0);
        }
        DMMsg.msgVDES.exptVel = this->cmd_.vel;
        memcpy(txBuf, &DMMsg.msgVDES.exptVel, 4);
        break;
    }
    case WorkMode_e::EMIT: {
        lenBuf = 8;
        DMMsg.msgEMIT.exptScale = this->cmd_.pos;
        DMMsg.msgEMIT.exptVelX100 = static_cast<uint16_t>(
                ((this->cmd_.vel < 0) ? -this->cmd_.vel : this->cmd_.vel) *
                100.f);
        DMMsg.msgEMIT.imaxX10000 = static_cast<uint16_t>(
                ((this->cmd_.torq < 0) ? -this->cmd_.torq : this->cmd_.torq) /
                status_.torqConstant / status_.currMax *
                status_.currTxCodeSpan);
        float f = DMMsg.msgEMIT.exptScale;
        memcpy(txBuf, &f, 4);
        txBuf[4] = static_cast<uint8_t>((DMMsg.msgEMIT.exptVelX100) >> 8);
        txBuf[5] = static_cast<uint8_t>(DMMsg.msgEMIT.exptVelX100);
        txBuf[6] = static_cast<uint8_t>((DMMsg.msgEMIT.imaxX10000) >> 8);
        txBuf[7] = static_cast<uint8_t>(DMMsg.msgEMIT.imaxX10000);
        break;
    }
    default: {
        LOG::error("DMMotor", " %s: this mode is not supported", this->name_);
        break;
    }
    }
    if (isMIT) {
        lenBuf = 8;
        txBuf[0] = static_cast<uint8_t>((DMMsg.msgMIT.exptScale & 0xFF00) >> 8);
        txBuf[1] = static_cast<uint8_t>(DMMsg.msgMIT.exptScale & 0x00FF);
        txBuf[2] = static_cast<uint8_t>((DMMsg.msgMIT.exptVel & 0x0FF0) >> 4);
        txBuf[3] = static_cast<uint8_t>((DMMsg.msgMIT.exptVel & 0x000F) << 4 |
                                        ((DMMsg.msgMIT.Kp & 0x0FF0) >> 8));
        txBuf[4] = static_cast<uint8_t>(DMMsg.msgMIT.Kp & 0x000F);
        txBuf[5] = static_cast<uint8_t>((DMMsg.msgMIT.Kd & 0x0FF0) >> 4);
        txBuf[6] = static_cast<uint8_t>(
                (DMMsg.msgMIT.Kd & 0x000F) << 4 |
                ((DMMsg.msgMIT.torqueOffset & 0x0F00) >> 8));
        txBuf[7] = static_cast<uint8_t>(DMMsg.msgMIT.torqueOffset & 0x00FF);
    }

    if ((this->cmd_.SW && !this->cmd_.prevSW) ||
        (this->cmd_.SW && errorCode_ == ErrorCode_e::MotorDisable)) {
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
        rslt |= this->send(this->ctrlId_, txBuf, lenBuf);
    }
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


MotorTypeDef_e DMMotor::registerReg(Reg_s *_regObj, RegValue_u *_regValue)
{
    if (_regObj == nullptr) {
        LOG::error("DMMotor", " %s: registerReg failed, _regObj is nullptr",
                   this->name_);
        return 1;
    }
    auto it = regObjList_.find(_regObj->regId);
    if (it != regObjList_.end()) {
        LOG::error(
                "DMMotor",
                " %s: registerReg failed, _regObj->regId is already registered",
                this->name_);
        return 1;
    }
    LOG::info("DMMotor", " %s: registerReg success", this->name_);
    regObjList_.insert({ _regObj->regId, _regObj });
    regValueList_.insert({ _regObj->regId, _regValue });
    return 0;
}

MotorTypeDef_e DMMotor::cancelReg(RegId_e regId)
{
    LOG::info("DMMotor", " %s: cancelReg success, regId:%d", this->name_,
              regId);
    regObjList_.erase(regId);
    return 0;
}

MotorTypeDef_e DMMotor::writeOneReg(RegId_e _regId, uint8_t dat[4])
{
    MotorTypeDef_e rslt = 0;
    // 报文ID : 0x7FF, D0 : CANID_L, D1 : CANID_H, D2 : 0x55, D3 : RID, D4 : dat1,
    // D5 : dat2, D6 : dat3, D7 : dat4
    auto it = regObjList_.find(_regId);
    if (it != regObjList_.end()) {
        (*it).second->isWrite = false;
        uint16_t id = static_cast<uint16_t>(canId());
        uint8_t writeTxBuffer[8] = //
                { static_cast<uint8_t>(id),
                  static_cast<uint8_t>(id >> 8),
                  0x55,
                  static_cast<uint8_t>(_regId),
                  dat[0],
                  dat[1],
                  dat[2],
                  dat[3] };
        this->send(0x7FF, writeTxBuffer, 8);
    }
    return rslt;
}

MotorTypeDef_e DMMotor::readOneReg(RegId_e _regId)
{
    MotorTypeDef_e rslt = 0;
    // 报文ID : 0x7FF, D0 : CANID_L, D1 : CANID_H, D2 : 0x33, D3 : RID, D4 : 0x00,
    // D5 : 0x00, D6 : 0x00, D7 : 0x00
    auto it = regObjList_.find(_regId);
    if (it != regObjList_.end()) {
        (*it).second->isRead = false;
        uint16_t id = canId();
        uint8_t readTxBuffer[8] = { static_cast<uint8_t>(id),
                                    static_cast<uint8_t>(id >> 8),
                                    0x33,
                                    static_cast<uint8_t>(_regId),
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00 };
        this->send(0x7FF, readTxBuffer, 8);
    }
    return rslt;
}

MotorTypeDef_e DMMotor::storageOneReg(RegId_e _regId)
{
    MotorTypeDef_e rslt = 0;
    // 报文ID : 0x7FF, D0 : CANID_L, D1 : CANID_H, D2 : 0xAA, D3 : RID, D4 : 0x00,
    // D5 : 0x00, D6 : 0x00, D7 : 0x00
    auto it = regObjList_.find(_regId);
    if (it != regObjList_.end()) {
        (*it).second->isStorage = false;
        uint16_t id = canId();
        uint8_t storageTxBuf[8] = { static_cast<uint8_t>(id),
                                    static_cast<uint8_t>(id >> 8),
                                    0xAA,
                                    static_cast<uint8_t>(_regId),
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00 };
        this->send(0x7FF, storageTxBuf, 8);
    }
    return rslt;
}

MotorTypeDef_e DMMotor::writeReg()
{
    static uint16_t writeWaitTime = 0;
    static auto it = regObjList_.begin();
    if (it != regObjList_.end()) {
        if ((*it).second->isWrite == true && writeWaitTime % 10 == 0) {
            writeOneReg((*it).first, (*it).second->dat);
            writeWaitTime++;
            it++;
            return 0;
        }
    } else {
        writeWaitTime = 0;
        it = regObjList_.begin();
        return 1;
    }
    if (writeWaitTime % 10 != 0)
        writeWaitTime++;
    return 0;
}

MotorTypeDef_e DMMotor::readReg()
{
    static uint16_t readWaitTime = 0;
    static auto it = regObjList_.begin();
    if (it != regObjList_.end()) {
        if ((*it).second->isRead == true && readWaitTime % 10 == 0) {
            readOneReg((*it).first);
            readWaitTime++;
            it++;
        }
    } else {
        readWaitTime = 0;
        it = regObjList_.begin();
        return 1;
    }
    if (readWaitTime % 10 != 0)
        readWaitTime++;
    return 0;
}

MotorTypeDef_e DMMotor::storageReg()
{
    static uint16_t storageWaitTime = 0;
    static auto it = regObjList_.begin();
    if (it != regObjList_.end()) {
        if ((*it).second->isStorage == true && storageWaitTime % 10 == 0) {
            storageOneReg((*it).first);
            storageWaitTime++;
            it++;
        }
    } else {
        storageWaitTime = 0;
        it = regObjList_.begin();
        return 1;
    }
    if (storageWaitTime % 10 != 0)
        storageWaitTime++;
    return 0;
}

MotorTypeDef_e DMMotor::updateRegDat()
{
    for (auto &regObj : this->regObjList_) {
        auto datCompare = [&]() -> bool {
            return std::equal(
                    std::begin(this->regValueList_[regObj.first]->dat),
                    std::end(this->regValueList_[regObj.first]->dat),
                    std::begin(this->preRegValue_[regObj.first]));
        };
        if (datCompare() == false) {
            memcpy(&regObj.second->dat, &this->regValueList_[regObj.first]->dat,
                   4);
            memcpy(&this->preRegValue_[regObj.first],
                   &this->regValueList_[regObj.first]->dat, 4);
            regObj.second->isWrite = true;
        }
    }
    return 0;
}
