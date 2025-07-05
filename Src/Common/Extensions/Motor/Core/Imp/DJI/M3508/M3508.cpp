#include "M3508.hpp"

#include "DJIMotor.hpp"

#include <cstring>

using namespace PINYMOTOR;
M3508::M3508(const char _name[16], InitConfig_s _config)
        : DJIMotor(_name, _config)
{
    strcpy(this->model_.name, "DJI-M3508");
    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = (3591.f / 187.f);
    this->model_.rxBaseId = 0x200;

    if (_config.offsetId > 3)
        this->model_.txBaseId = 0x1FF;
    else
        this->model_.txBaseId = 0x200;

    this->stats_ = DJIMotorStats_s(M3508_VOLT_TX_CODE_SPAN, // voltTxCodeSpan
                                   M3508_CURR_TX_CODE_SPAN, // currTxCodeSpan
                                   M3508_CURR_RX_CODE_SPAN, // currRxCodeSpan
                                   M3508_CURR_RATED,        // currRated
                                   M3508_TORQ_RATED,        // torqRated
                                   M3508_VOLT_MAX,          // voltMax
                                   M3508_CURR_MAX,          // currMax
                                   M3508_TORQ_MAX,          // torqMax
                                   M3508_TORQ_CONSTANT      // torqConstant
    );

    this->registerMotor();
    this->updateMotorMap();
    this->registerRecvCallback();
    this->updateCtrlId();

    checkBaseConfig();

    this->log("INFO", "green", "Motor %s: An instance of DJIMotor created",
              this->name_);
}

MotorTypeDef_e M3508::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ != ComType_e ::CAN) {
        rslt |= 1;
        this->log("ERROR", "red", "M3508 %s: only support CAN comtype",
                  this->name_);
    }

    if (this->workMode_ != WorkMode_e::QUAD_CURR) {
        rslt |= 1;
        this->log("ERROR", "red", "M3508 %s: WorkMode is not supported",
                  this->name_);
    }

    if (this->offsetId_ > 8) {
        rslt |= 1;
        this->log("ERROR", "red", "M3508 %s: Max Offset ID is only 8!",
                  this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        this->log("ERROR", "red", "M3508 %s: Max TxFreq is only 1000!",
                  this->name_);
    }

    return rslt;
}
