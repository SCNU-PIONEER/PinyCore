#include "DM4340.hpp"

#include "DMMotor.hpp"

#include "StmLog.hpp"

#include <cstring>

using namespace PINYMOTOR;
using namespace DMMOTOR;
DM4340::DM4340(const char _name[16], InitConfig_s _config)
        : DMMotor(_name, std::move(_config))
{
    LOG::CHECK(checkBaseConfig());

    strcpy(this->model_.name, "DM-DM4340");
    this->model_.measureMax = 16383;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 1.f;
    this->model_.rxBaseId = 0x10;
    this->model_.txBaseId = 0x00;

    this->status_ = Status_s(P_MAX,             // PMax
                             V_MAX,             // VMax
                             T_MAX,             // TMax
                             MIT_KP_MAX,        // MITKpMax
                             MIT_KD_MAX,        // MITKdMax
                             CURR_TX_CODE_SPAN, // currTxCodeSpan
                             CURR_RATED,        // currRated
                             TORQ_RATED,        // torqRated
                             CURR_MAX,          // currMax
                             TORQ_MAX,          // torqMax
                             TORQ_CONSTANT      // torqConstant
    );

    this->registerMotor();
    this->registerRecvCallback();
    this->updateCtrlId();

    LOG::info("DM4340",
              " %s: An instance of DM4340 created, rxBaseId:%hx, txBaseId:%hx",
              this->name_, this->model_.rxBaseId, this->model_.txBaseId);
    // TODO:
}

MotorTypeDef_e DM4340::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ != ComType_e::FDCAN &&
        this->comType_ != ComType_e::CAN) {
        rslt |= 1;
        LOG::error("DM4340", " %s: only support FDCAN or CAN comtype",
                   this->name_);
    }

    if (this->workMode_ == WorkMode_e::QUAD_VOLT) {
        rslt |= 1;
        LOG::error("DM4340", " %s: WorkMode is not supported", this->name_);
    }

    if (this->offsetId_ > 9) {
        rslt |= 1;
        LOG::error("DM4340", " %s: Max Offset ID is only 9!",

                   this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        LOG::error("DM4340", " %s: Max TxFreq is only 1000!", this->name_);
    }
    return rslt;
}
