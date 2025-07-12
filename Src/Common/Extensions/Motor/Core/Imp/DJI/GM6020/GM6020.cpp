#include "GM6020.hpp"

#include "DJIMotor.hpp"

#include <cstring>

using namespace PINYMOTOR;
GM6020::GM6020(const char _name[16], InitConfig_s _config)
        : DJIMotor(_name, _config)
{
    strcpy(this->model_.name, "DJI-GM6020");
    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 1.f;
    this->model_.rxBaseId = 0x204;

    if (this->workMode_ == WorkMode_e::QUAD_CURR) {
        if (_config.offsetId > 4)
            this->model_.txBaseId = 0x2FE;
        else
            this->model_.txBaseId = 0x1FE;
    } else if (this->workMode_ == WorkMode_e::QUAD_VOLT) {
        if (_config.offsetId > 4)
            this->model_.txBaseId = 0x2FF;
        else
            this->model_.txBaseId = 0x1FF;
    }

    this->stats_ = DJIMotorStats_s(GM6020_VOLT_TX_CODE_SPAN, // voltTxCodeSpan
                                   GM6020_CURR_TX_CODE_SPAN, // currTxCodeSpan
                                   GM6020_CURR_RX_CODE_SPAN, // currRxCodeSpan
                                   GM6020_CURR_RATED,        // currRated
                                   GM6020_TORQ_RATED,        // torqRated
                                   GM6020_VOLT_MAX,          // voltMax
                                   GM6020_CURR_MAX,          // currMax
                                   GM6020_TORQ_MAX,          // torqMax
                                   GM6020_TORQ_CONSTANT      // torqConstant
    );

    this->registerMotor();
    this->updateMotorMap();
    this->registerRecvCallback();
    this->updateCtrlId();

    checkBaseConfig();
    log.info(LOCATION, "GM6020",
             " %s: An instance of GM6020 created, rxBaseId:%hx, txBaseId:%hx",
             this->name_, this->model_.rxBaseId, this->model_.txBaseId);
}

MotorTypeDef_e GM6020::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ != PINYMOTOR::ComType_e ::CAN) {
        rslt |= 1;
        log.error(LOCATION, "GM6020", " %s: only support CAN comtype",
                  this->name_);
    }

    if (!(this->workMode_ == WorkMode_e::QUAD_CURR ||
          this->workMode_ == WorkMode_e::QUAD_VOLT)) {
        rslt |= 1;
        log.error(LOCATION, "GM6020", "%s: WorkMode is not supported",
                  this->name_);
    }

    if (this->offsetId_ > 7) {
        rslt |= 1;
        log.error(LOCATION, "GM6020", "%s: Max Offset ID is only 7!",
                  this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        log.error(LOCATION, "GM6020", " %s: Max TxFreq is only 1000!",
                  this->name_);
    }

    return rslt;
}
