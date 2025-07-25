#include "DM4310.hpp"

#include "DMMotor.hpp"

#include "StmLog.hpp"

#include <cstring>

using namespace PINYMOTOR;
using namespace DMMOTOR;
DM4310::DM4310(const char _name[16], InitConfig_s _config)
        : DMMotor(_name, std::move(_config))
{
    strcpy(this->model_.name, "DM-DM4310");
    this->model_.measureMax = 16383;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 1.f;
    this->model_.rxBaseId = 0x10;
    this->model_.txBaseId = 0x00;

    this->status_ = Status_s(DM4310_P_MAX,             // PMax
                             DM4310_V_MAX,             // VMax
                             DM4310_T_MAX,             // TMax
                             DM4310_MITKp_MAX,         // MITKpMax
                             DM4310_MITKd_MAX,         // MITKdMax
                             DM4310_CURR_TX_CODE_SPAN, // currTxCodeSpan
                             DM4310_CURR_RATED,        // currRated
                             DM4310_TORQ_RATED,        // torqRated
                             DM4310_CURR_MAX,          // currMax
                             DM4310_TORQ_MAX,          // torqMax
                             DM4310_TORQ_CONSTANT      // torqConstant
    );

    this->registerMotor();
    this->registerRecvCallback();
    this->updateCtrlId();

    checkBaseConfig();

    LOG::info("DM4310",
              " %s: An instance of DM4310 created, rxBaseId:%hx, txBaseId:%hx",
              this->name_, this->model_.rxBaseId, this->model_.txBaseId);
    // TODO:
}

MotorTypeDef_e DM4310::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ != ComType_e::FDCAN &&
        this->comType_ != ComType_e::CAN) {
        rslt |= 1;
        LOG::error("DM4310", " %s: only support FDCAN or CAN comtype",
                   this->name_);
    }

    if (this->workMode_ == WorkMode_e::QUAD_VOLT) {
        rslt |= 1;
        LOG::error("DM4310", " %s: WorkMode is not supported", this->name_);
    }

    if (this->offsetId_ > 9) {
        rslt |= 1;
        LOG::error("DM4310", " %s: Max Offset ID is only 9!", this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        LOG::error("DM4310", " %s: Max TxFreq is only 1000!", this->name_);
    }

    return rslt;
}
