#include "M3508.hpp"

#include "DJIMotor.hpp"

#include "StmLog.hpp"

#include <cstring>

using namespace PINYMOTOR;
using namespace DJIMOTOR;
M3508::M3508(const char _name[16], InitConfig_s _config)
        : DJIMotor(_name, std::move(_config))
{
    LOG::CHECK(checkBaseConfig());

    strcpy(this->model_.name, "DJI-M3508");
    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = (3591.f / 187.f);
    this->model_.rxBaseId = 0x200;

    if (_config.offsetId > 4)
        this->model_.txBaseId = 0x1FF;
    else
        this->model_.txBaseId = 0x200;

    this->status_ =
            Status_s(VOLT_TX_CODE_SPAN, // voltTxCodeSpan
                     CURR_TX_CODE_SPAN, // currTxCodeSpan
                     CURR_RX_CODE_SPAN, // currRxCodeSpan
                     CURR_RATED,        // currRated
                     TORQ_RATED,        // torqRated
                     VOLT_MAX,          // voltMax
                     20.f,     // currMax BUG: we need C620 MAX_CURRENT value
                     TORQ_MAX, // torqMax
                     TORQ_CONSTANT // torqConstant
            );

    this->registerMotor();
    this->updateMotorMap();
    this->registerRecvCallback();
    this->updateCtrlId();

    LOG::info("M3508",
              " %s: An instance of M3508 created, rxBaseId:%hx, txBaseId:%hx",
              this->name_, this->model_.rxBaseId, this->model_.txBaseId);
}

MotorTypeDef_e M3508::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ != ComType_e ::CAN) {
        rslt |= 1;
        LOG::error("M3508", " %s: only support CAN comtype", this->name_);
    }

    if (this->workMode_ != WorkMode_e::QUAD_CURR) {
        rslt |= 1;
        LOG::error("M3508", " %s: WorkMode is not supported", this->name_);
    }

    if (this->offsetId_ > 8) {
        rslt |= 1;
        LOG::error("M3508", " %s: Max Offset ID is only 8!", this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        LOG::error("M3508", " %s: Max TxFreq is only 1000!", this->name_);
    }

    return rslt;
}
