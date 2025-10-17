#include "DMMotor.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace DMMOTOR;

MotorTypeDef_e DMMotor::registerReg(Reg_s *_regObj, RegValue_u *_regValue)
{
    if (_regObj == nullptr) {
        LOG::error("DMMotor", " %s: registerReg failed, _regObj is nullptr",
                   regInfo_.name);
        return 1;
    }
    auto it = regObjList_.find(_regObj->regId);
    if (it != regObjList_.end()) {
        LOG::error(
                "DMMotor",
                " %s: registerReg failed, _regObj->regId is already registered",
                regInfo_.name);
        return 1;
    }
    LOG::info("DMMotor", " %s: registerReg success", regInfo_.name);
    regObjList_.insert({ _regObj->regId, _regObj });
    regValueList_.insert({ _regObj->regId, _regValue });
    return 0;
}

MotorTypeDef_e DMMotor::cancelReg(RegId_e _regId)
{
    LOG::info("DMMotor", " %s: cancelReg success, regId:%d", regInfo_.name,
              _regId);
    regObjList_.erase(_regId);
    return 0;
}

MotorTypeDef_e DMMotor::writeOneReg(RegId_e _regId, uint8_t _dat[4])
{
    MotorTypeDef_e rslt = 0;
    // 报文ID : 0x7FF, D0 : CANID_L, D1 : CANID_H, D2 : 0x55, D3 : RID, D4 : dat1,
    // D5 : dat2, D6 : dat3, D7 : dat4
    auto it = regObjList_.find(_regId);
    if (it != regObjList_.end()) {
        (*it).second->isWrite = false;
        uint16_t id = canId();
        uint8_t writeTxBuffer[8] = //
                { static_cast<uint8_t>(id),
                  static_cast<uint8_t>(id >> 8),
                  0x55,
                  static_cast<uint8_t>(_regId),
                  _dat[0],
                  _dat[1],
                  _dat[2],
                  _dat[3] };
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

MotorTypeDef_e DMMotor::updateRegDat()
{
    // TODO: thread safe
    for (auto &regObj : this->regObjList_) {
        auto datCompare = [&]() -> bool {
            return std::equal(
                    std::begin(this->regValueList_[regObj.first]->dat),
                    std::end(this->regValueList_[regObj.first]->dat),
                    std::begin(this->preRegValue_[regObj.first]));
        };
        if (!datCompare()) {
            memcpy(&regObj.second->dat, &this->regValueList_[regObj.first]->dat,
                   4);
            memcpy(&this->preRegValue_[regObj.first],
                   &this->regValueList_[regObj.first]->dat, 4);
            regObj.second->isWrite = true;
        }
    }
    return 0;
}