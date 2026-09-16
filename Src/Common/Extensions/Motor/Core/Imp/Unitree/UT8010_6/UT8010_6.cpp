#include "UT8010_6.hpp"
#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace UTMOTOR;

UT80106::UT80106(const char _name[16], InitConfig_s _config, UART_HandleTypeDef *_huart, WorkMode_e _workMode)
        : UTMotor(_name, _config, _huart, _workMode)
{
    LOG::CHECK([this]() { return checkBaseConfig(); });

    regInfo_.model.measureMax = 0;
    regInfo_.model.measureMin = 0;
    regInfo_.model.reductionRatio = RR;
    regInfo_.model.rxBaseId = 0;
    regInfo_.model.txBaseId = 0;
    this->ctrlId_ = _config.offsetId;

    Status_s status = {};
    status.PMax = P_MAX;
    status.VMax = V_MAX;
    status.TMax = T_MAX;
    status.KpMax = KP_MAX;
    status.KdMax = KD_MAX;
    status.currMax = CURR_MAX;
    status.torqMax = TRQE_MAX;
    status.speedMax = SPEED_MAX;
    status.Kn = KN;
    this->status_ = status;

    this->registerRecvCallback();

    LOG::info("UT8010_6", " %s: An instance of UT8010_6 created, ctrlId:0x%hx", regInfo_.name, this->ctrlId_);
}

MotorTypeDef_e UT80106::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (regInfo_.comType != PINYMOTOR::ComType_e ::RS485) {
        rslt |= 1;
        LOG::error("UT8010_6", " %s: only support RS485 comtype", regInfo_.name);
    }

    if (this->workMode_ != WorkMode_e::EMIT) {
        rslt |= 1;
        LOG::error("UT8010_6", " %s: WorkMode only support EMIT", regInfo_.name);
    }

    if (regInfo_.offsetId > 15) {
        rslt |= 1;
        LOG::error("UT8010_6", " %s: Max Offset ID is only 15!", regInfo_.name);
    }

    if (AUX_.txFreq > 1000) {
        rslt |= 1;
        LOG::error("UT8010_6", " %s: Max TxFreq is only 1000!", regInfo_.name);
    }

    return rslt;
}
