#include "DMMotor.hpp"

#include "Bsp_can.hpp"

#include "MotorCommonMacros.hpp"

using namespace PINYMOTOR;

template <typename T> void DMMotor<T>::registerRecvCallback()
{
    // lamda
    Can::instance().registerCallback(
            static_cast<canHandle *>(this->pComHandle_), this->masterId(),
            [this](uint8_t *_rxBuf) {
                // basic cb
                this->_parse_(_rxBuf);
                // user cb
                if (this->userRecvCallback_ != nullptr) {
                    this->userRecvCallback_(_rxBuf);
                }
            });
    this->log("INFO", "", "Motor %s: Receive cb registed", this->name_);
}

template <typename T>
MotorTypeDef_e DMMotor<T>::_cmd_(MotorCmdType_e _cmd, float _cmdData)
{
    switch (_cmd) {
    case MotorCmdType_e::SET_SPD:
        cmd_.speed = _cmdData;
        break;
    case MotorCmdType_e::SET_POS:
        cmd_.pos = _cmdData;
        break;
    case MotorCmdType_e::SET_TORQ:
        cmd_.torq = _cmdData;
        break;
    default:
        this->log("ERROR", "", "Motor %s: Invalid cmd type", this->name_);
        return 1;
    };
    return 0;
}

template <typename T> MotorTypeDef_e DMMotor<T>::_cmd_(MotorCmdType_e _cmd)
{
    if (_cmd == MotorCmdType_e::EN) {
        cmd_.updateSW(true);
    } else if (_cmd == MotorCmdType_e::DIS) {
        cmd_.updateSW(false);
    } else {
        this->log("ERROR", "", "Motor %s: not SW cmd!", this->name_);
        return 1;
    }
    return 0;
}

template <typename T>
MotorTypeDef_e DMMotor<T>::_send_(uint8_t *_txBuf, uint8_t _len)
{
    return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
            static_cast<canHandle *>(this->pComHandle_), this->ctrlId_, _txBuf,
            _len));
}

template <typename T> MotorTypeDef_e DMMotor<T>::_parse_(uint8_t *_rxBuf)
{
    // 先处理非常规数据反馈的帧
    if (_rxBuf[0] == static_cast<uint8_t>(canId()) &&
        _rxBuf[1] == static_cast<uint8_t>(canId() >> 8)) {
        uint32_t rawDat = (_rxBuf[7] << 24) | (_rxBuf[6] << 16) |
                          (_rxBuf[5] << 8) | _rxBuf[4];
        auto it = regObjList_.find(static_cast<DMMotorRegId_e>(_rxBuf[3]));
        if (_rxBuf[2] == 0x33) {
            // 读反馈
            (*it).second->isRead = true;
            memcpy(&(*it).second->dat, &rawDat, 4);
        } else if (_rxBuf[2] == 0x55) {
            // 写反馈
            (*it).second->isWrite = true;
        } else if (_rxBuf[2] == 0xAA) {
            // 存储反馈
            (*it).second->isStorage = true;
        } else {
            this->log("ERROR", "", "Motor %s: Unknown feedback type",
                      this->name_);
            return 1;
        }
    } else {
        DMMotorFeedback_s fb;
        fb.ID = _rxBuf[0] & 0x0F;
        fb.errorCode = static_cast<DMMotorErrorCode_e>(_rxBuf[0] >> 4);
        fb.rawScale = (_rxBuf[1] << 8) | _rxBuf[2];
        fb.rawVel = (_rxBuf[3] << 4) | (_rxBuf[4] >> 4);
        fb.torque = ((_rxBuf[4] & 0xF) << 8 | _rxBuf[5]);
        fb.mosTemperature = _rxBuf[6];
        fb.rotorTemperature = _rxBuf[7];

        errorCode_ = fb.errorCode;

        this->data_.rawScale = fb.rawScale;

        this->data_.spdRadps =
                uint2float(fb.rawVel, -stats_.VMax, stats_.VMax, 12) /
                this->RR();
        this->data_.spdRpm = radps2rpm(this->data_.spdRadps);
        this->data_.torq =
                uint2float(fb.torque, -stats_.TMax, stats_.TMax, 12) *
                this->RR();

        this->data_.curr = this->data_.torq / stats_.torqConstant;

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

template <typename T> MotorTypeDef_e DMMotor<T>::_ctrl_()
{
    MotorTypeDef_e rslt = 0;
    typedef union {
        DMMITMsg_s msgMIT;
        DMPDESVDESMsg_s msgPDESVDES;
        DMVDESMsg_s msgVDES;
        DMEMITMsg_s msgEMIT;
    } DMMsg_u;
    DMMsg_u DMMsg = { 0 };
    uint8_t txBuf[8] = { 0 };
    uint8_t lenBuf = 0;
    bool isMIT = false;
    switch (this->workMode_) {
    case WorkMode_e::QUAD_CURR: {
        this->log("ERROR", "", "Motor %s: QUAD_CURR mode is not supported",
                  this->name_);
        break;
    }
    case WorkMode_e::QUAD_VOLT: {
        this->log("ERROR", "", "Motor %s: QUAD_VOLT mode is not supported",
                  this->name_);
        break;
    }
    case WorkMode_e::MIT_TT: {
        DMMsg.msgMIT.torqueOffset =
                float2uint(cmd_.torq, -stats_.TMax, stats_.TMax, 12);
        DMMsg.msgMIT.Kp = 0;
        DMMsg.msgMIT.Kd = 0;
        isMIT = true;
        break;
    }
    case WorkMode_e::MIT_VDESPDES: {
        DMMsg.msgMIT.exptScale =
                float2uint(cmd_.pos, -stats_.PMax, stats_.PMax, 16);
        DMMsg.msgMIT.exptVel =
                float2uint(cmd_.speed, -stats_.VMax, stats_.VMax, 12);
        DMMsg.msgMIT.Kd =
                float2uint(this->MITKd_, -stats_.MITKdMax, stats_.MITKdMax, 12);
        DMMsg.msgMIT.Kp =
                float2uint(this->MITKp_, -stats_.MITKpMax, stats_.MITKpMax, 12);
        DMMsg.msgMIT.torqueOffset =
                float2uint(cmd_.torq, -stats_.TMax, stats_.TMax, 12);
        isMIT = true;
        break;
    }
    case WorkMode_e::MIT_VDES: {
        DMMsg.msgMIT.exptVel =
                float2uint(cmd_.speed, -stats_.VMax, stats_.VMax, 12);
        DMMsg.msgMIT.Kd =
                float2uint(this->MITKd_, -stats_.MITKdMax, stats_.MITKdMax, 12);
        DMMsg.msgMIT.Kp = 0;
        DMMsg.msgMIT.torqueOffset =
                float2uint(cmd_.torq, -stats_.TMax, stats_.TMax, 12);
        isMIT = true;
        break;
    }
    case WorkMode_e::PDESVDES: {
        lenBuf = 8;
        this->ctrlId_ = canId() + 0x100;
        DMMsg.msgPDESVDES.exptScale = cmd_.pos;
        DMMsg.msgPDESVDES.exptVel = cmd_.speed;
        memcpy(txBuf, &DMMsg.msgPDESVDES.exptScale, 4);
        memcpy(&txBuf[4], &DMMsg.msgPDESVDES.exptVel, 4);
        break;
    }
    case WorkMode_e::VDES: {
        lenBuf = 4;
        this->ctrlId_ = canId() + 0x200;
        DMMsg.msgVDES.exptVel = cmd_.speed;
        memcpy(txBuf, &DMMsg.msgVDES.exptVel, 4);
        break;
    }
    case WorkMode_e::EMIT: {
        lenBuf = 8;
        this->ctrlId_ = canId() + 0x300;
        DMMsg.msgEMIT.exptScale = cmd_.pos;
        DMMsg.msgEMIT.exptVelX100 = static_cast<uint16_t>(
                ((cmd_.speed < 0) ? -cmd_.speed : cmd_.speed) * 100.f);
        DMMsg.msgEMIT.imaxX10000 = static_cast<uint16_t>(
                ((cmd_.torq < 0) ? -cmd_.torq : cmd_.torq) /
                stats_.torqConstant / stats_.currMax * stats_.currTxCodeSpan);
        float f = DMMsg.msgEMIT.exptScale;
        memcpy(txBuf, &f, 4);
        txBuf[4] = static_cast<uint8_t>((DMMsg.msgEMIT.exptVelX100) >> 8);
        txBuf[5] = static_cast<uint8_t>(DMMsg.msgEMIT.exptVelX100);
        txBuf[6] = static_cast<uint8_t>((DMMsg.msgEMIT.imaxX10000) >> 8);
        txBuf[7] = static_cast<uint8_t>(DMMsg.msgEMIT.imaxX10000);
        break;
    }
    default: {
        this->log("ERROR", "", "Motor %s: this mode is not supported",
                  this->name_);
        break;
    }
    }
    if (isMIT) {
        lenBuf = 8;
        this->ctrlId_ = canId();
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

    if (this->checkSend()) {
        this->lastSendTick = xTaskGetTickCount();
        if ((cmd_.SW && !cmd_.prevSW) ||
            (cmd_.SW && errorCode_ == DMMotorErrorCode_e::MotorDisable)) {
            this->enable();
        } else if (!cmd_.SW) {
            this->disable();
        } else {
            rslt |= this->send(txBuf, lenBuf);
        }
    }
    return rslt;
}

template <typename T> MotorTypeDef_e DMMotor<T>::enable()
{
    MotorTypeDef_e rslt = 0;
    // 定义一个8字节的数组enableCmdPack，用于存储使能命令
    uint8_t enableCmdPack[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC
    };
    rslt |= this->send(enableCmdPack, 8);
    cmd_.updateSW(true); // force enable
    return rslt;
}

template <typename T> MotorTypeDef_e DMMotor<T>::disable()
{
    MotorTypeDef_e rslt = 0;
    // 定义一个8字节的数组disableCmdPack，用于存储禁用命令
    uint8_t disableCmdPack[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD
    };
    rslt |= this->send(disableCmdPack, 8);
    cmd_.updateSW(false); // force disable
    return rslt;
}

template <typename T>
MotorTypeDef_e DMMotor<T>::registerReg(DMMotorReg_s *_regObj)
{
    if (_regObj == nullptr) {
        this->log("ERROR", "",
                  "Motor %s: registerReg failed, _regObj is nullptr",
                  this->name_);
        return 1;
    }
    auto it = regObjList_.find(_regObj->regId);
    if (it != regObjList_.end()) {
        this->log(
                "ERROR", "",
                "Motor %s: registerReg failed, _regObj->regId is already registered",
                this->name_);
        return 1;
    }
    this->log("INFO", "", "Motor %s: registerReg success", this->name_);
    regObjList_.insert({ _regObj->regId, _regObj });
    return 0;
}

template <typename T> MotorTypeDef_e DMMotor<T>::cancelReg(DMMotorRegId_e regId)
{
    this->log("INFO", "", "Motor %s: cancelReg success", this->name_);
    regObjList_.erase(regId);
    return 0;
}

template <typename T>
MotorTypeDef_e DMMotor<T>::writeReg(DMMotorRegId_e _regId, uint8_t dat[4])
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
        rslt |= static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                this->pComHandle_, 0x7FF, writeTxBuffer, 8));
    }
    return rslt;
}

template <typename T> MotorTypeDef_e DMMotor<T>::readReg(DMMotorRegId_e _regId)
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
        rslt |= static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                this->pComHandle_, 0x7FF, readTxBuffer, 8));
    }
    return rslt;
}

template <typename T>
MotorTypeDef_e DMMotor<T>::storageReg(DMMotorRegId_e _regId)
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
        rslt |= static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                this->pComHandle_, 0x7FF, storageTxBuf, 8));
    }
    return rslt;
}
