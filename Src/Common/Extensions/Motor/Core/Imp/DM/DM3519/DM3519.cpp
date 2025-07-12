#include "DM3519.hpp"

#include "DMMotor.hpp"
#include <cstring>

using namespace PINYMOTOR;
DM3519::DM3519(const char _name[16], InitConfig_s _config)
        : DMMotor(_name, _config)
{
    strcpy(this->model_.name, "DM-DM3519");
    this->model_.measureMax = 16383;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = (3591.f / 187.f);
    this->model_.rxBaseId = 0x10;
    this->model_.txBaseId = 0x00;

    this->stats_ = DMMotorStats_s{
        DM3519_P_MAX,             // PMax
        DM3519_V_MAX,             // VMax
        DM3519_T_MAX,             // TMax
        DM3519_MITKp_MAX,         // MITKpMax
        DM3519_MITKd_MAX,         // MITKdMax
        DM3519_CURR_TX_CODE_SPAN, // currTxCodeSpan
        DM3519_CURR_RATED,        // currRated
        DM3519_TORQ_RATED,        // torqRated
        DM3519_CURR_MAX,          // currMax
        DM3519_TORQ_MAX,          // torqMax
        DM3519_TORQ_CONSTANT      // torqConstant
    };

    this->registerMotor();
    this->registerRecvCallback();
    this->updateCtrlId();

    checkBaseConfig();

    log.info(LOCATION, "DM3519",
             " %s: An instance of DM3519 created, rxBaseId:%hx, txBaseId:%hx",
             this->name_, this->model_.rxBaseId, this->model_.txBaseId);
    // TODO:
}

MotorTypeDef_e DM3519::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ != ComType_e::FDCAN &&
        this->comType_ != ComType_e::CAN) {
        rslt |= 1;
        log.error(LOCATION, "DM3519", " %s: only support FDCAN or CAN comtype",
                  this->name_);
    }
    if (this->workMode_ == WorkMode_e::QUAD_VOLT) {
        rslt |= 1;
        log.error(LOCATION, "DM3519", " %s: WorkMode is not supported",
                  this->name_);
    }

    if (this->offsetId_ > 9) {
        rslt |= 1;
        log.error(LOCATION, "DM3519", " %s: Max Offset ID is only 9!",
                  this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        log.error(LOCATION, "DM3519", " %s: Max TxFreq is only 1000!",
                  this->name_);
    }

    return rslt;
}
