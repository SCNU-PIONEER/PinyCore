#include "IMotor.hpp"

#include "MotorManager.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;

IMotor::IMotor(const char _name[16], InitConfig_s _config)
        : id_(MotorManager::instance()->motorListSize())
        , pComHandle_(_config.pComHandle)
        , comType_(_config.comType)
        , workMode_(_config.workMode)
        , globalState_(GlobalState_e::UNREGISTER)
        , offsetId_(_config.offsetId)
        , txFreq_(_config.txFreq)
        , posPID_(std::move(_config.posPID))
        , velPID_(std::move(_config.velPID))
        , torqPID_(std::move(_config.torqPID))
        , cmdQueue_(xQueueCreate(3, sizeof(CmdBus_s)))
{
    strcpy(this->name_, _name);

    this->cmd_.clear();
    memset(&data_, 0, sizeof(Data_s));
}

bool IMotor::checkSend()
{
    if ((xTaskGetTickCount() - lastSendTick) >=
        pdMS_TO_TICKS(1000.f / this->txFreq_)) {
        this->lastSendTick = xTaskGetTickCount();
        return true;
    } else {
        return false;
    }
}

void IMotor::calcRecvFreq()
{
    uint32_t dt = xTaskGetTickCount() - lastRecvTick;
    lastRecvTick = xTaskGetTickCount();
    if (dt == 0) {
        return;
    } else {
        this->rxFreq_ = 1000.f / static_cast<float>(dt);
    }
}

MotorTypeDef_e IMotor::registerMotor()
{
    MotorManager *motorManager = MotorManager::instance();
    auto it = motorManager->motors().find(uid());
    if (it != motorManager->motors().end()) {
        return false;
    }
    motorManager->motors().insert({ uid(), this });
    return 0;
}

MotorTypeDef_e IMotor::cancelMotor()
{
    MotorManager *motorManager = MotorManager::instance();
    auto it = motorManager->motors().find(uid());
    if (it == motorManager->motors().end()) {
        return false;
    }
    motorManager->motors().erase(it);
    return 0;
}

MotorTypeDef_e IMotor::cmd(MotorCmdType_e _cmd, float _cmdData)
{
    if (_cmd != MotorCmdType_e::ON && _cmd != MotorCmdType_e::OFF &&
        _cmd != MotorCmdType_e::SET_MIT) {
        CmdBus_s cmd = { .cmdType = _cmd, .cmdVal1 = _cmdData };
        if (xQueueSend(cmdQueue_, &cmd, 0) != pdPASS) {
            LOG::warn("IMotor", " %s: cmdQueue send failed", this->name_);
            return 1;
        }
        return 0;
    } else {
        LOG::error("IMotor",
                   " %s: cmd ON or OFF is not supported in this function",
                   this->name_);
        return 1;
    }
}

MotorTypeDef_e IMotor::cmd(MotorCmdType_e _cmd)
{
    if (_cmd == MotorCmdType_e::ON || _cmd == MotorCmdType_e::OFF) {
        CmdBus_s cmd = { .cmdType = _cmd };
        if (xQueueSend(cmdQueue_, &cmd, 0) != pdPASS) {
            LOG::warn("IMotor", " %s: cmdQueue send failed", this->name_);
            return 1;
        }
        return 0;
    } else {
        LOG::error(
                "IMotor",
                " %s: none of cmd are supported in this function except ON and OFF",
                this->name_);
        return 1;
    }
}

MotorTypeDef_e IMotor::cmdMIT(float _pos, float _vel, float _torq)
{
    CmdBus_s cmd = { .cmdType = MotorCmdType_e::SET_MIT,
                     .cmdVal1 = _pos,
                     .cmdVal2 = _vel,
                     .cmdVal3 = _torq };
    if (xQueueSend(cmdQueue_, &cmd, 0) != pdPASS) {
        LOG::warn("IMotor", " %s: cmdQueue send failed", this->name_);
        return 1;
    }
    return 0;
}

MotorTypeDef_e IMotor::clampVel(float _velMax)
{
    this->cmd_.velMax = _velMax;
    return 0;
}

void IMotor::parseCmd()
{
    this->cmd_.curCmdType = cmdBuf_.cmdType;
    if (cmdBuf_.cmdType == MotorCmdType_e::SET_MIT) {
        this->cmd_.pos = cmdBuf_.cmdVal1;
        this->cmd_.vel = cmdBuf_.cmdVal2;
        this->cmd_.torq = cmdBuf_.cmdVal3;
    } else {
        switch (this->cmd_.curCmdType) {
        case MotorCmdType_e::ON: {
            this->cmd_.updateSW(true);
            break;
        }
        case MotorCmdType_e::OFF: {
            this->cmd_.updateSW(false);
            break;
        }
        case MotorCmdType_e::SET_ELEC: {
            this->cmd_.elec = cmdBuf_.cmdVal1;
            break;
        }
        case MotorCmdType_e::SET_TORQ: {
            this->cmd_.torq = cmdBuf_.cmdVal1;
            break;
        }
        case MotorCmdType_e::SET_VEL: {
            this->cmd_.vel = cmdBuf_.cmdVal1;
            break;
        }
        case MotorCmdType_e::SET_POS: {
            this->cmd_.pos = cmdBuf_.cmdVal1;
            break;
        }
        default: {
            LOG::error("IMotor", " %s: cmd type is not supported", this->name_);
        }
        }
    }
    if (!(this->cmd_.velMax < 0.f)) {
        this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                    this->cmd_.velMax);
    }
}

uint8_t IMotor::id() const { return id_; }

const Data_s &IMotor::data() const { return data_; }

void IMotor::setZeroAng()
{
    data_.zeroAng = data_.rawAng;
    float del = this->data_.rawAng - this->data_.zeroAng;
    this->data_.ang = del < 0 ? del + (2.f * std::numbers::pi_v<float>) : del;
    data_.multipCirAng = data_.singleCirAng = 0;
}

float IMotor::getCmdCurr()
{
    if (this->workMode_ == WorkMode_e::QUAD_VOLT) {
        // TODO: RLS volt ctrl
        return 0.f;
    } else {
        return cmd_.elec;
    }
}

float IMotor::txBaseId() const { return static_cast<float>(model_.txBaseId); }

float IMotor::rxBaseId() const { return static_cast<float>(model_.rxBaseId); }

float IMotor::rr() const { return model_.reductionRatio; }

float IMotor::measureMax() const
{
    return static_cast<float>(model_.measureMax);
}

float IMotor::measureMin() const
{
    return static_cast<float>(model_.measureMin);
}

float IMotor::span() const
{
    return static_cast<float>(model_.measureMax - model_.measureMin);
}

float IMotor::txFreq() const { return txFreq_; }

float IMotor::rxFreq() const { return rxFreq_; }

float IMotor::ang() const { return data_.ang; }

float IMotor::center() const { return data_.zeroAng; }

float IMotor::pos() const { return data_.multipCirAng; }

float IMotor::posNorm() const { return data_.singleCirAng; }

float IMotor::vel() const { return data_.spdRadps; }

float IMotor::torq() const { return data_.torq; }

void IMotor::overrideReductionRatio(float _newReductionRatio)
{
    model_.reductionRatio = _newReductionRatio;
}

void IMotor::overrideMeasureMax(float _newMeasureMax)
{
    model_.measureMax = static_cast<uint16_t>(_newMeasureMax);
}

void IMotor::overrideMeasureMin(float _newMeasureMin)
{
    model_.measureMin = static_cast<uint16_t>(_newMeasureMin);
}

const char *IMotor::getName() const { return name_; }
