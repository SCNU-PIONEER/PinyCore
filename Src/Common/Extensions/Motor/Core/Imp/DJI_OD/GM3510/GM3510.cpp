#include "GM3510.hpp"

#include "DJIOldMotor.hpp"

#include "StmLog.hpp"

#include <cstring>

using namespace PINYMOTOR;
using namespace DJI_ODMOTOR;

GM3510::GM3510(const char _name[16], InitConfig_s _config)
        : DJIOldMotor(_name, _config)
{
    LOG::CHECK(checkBaseConfig());

    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 1.f;
    this->model_.rxBaseId = 0x204;

    this->model_.txBaseId = 0x1FF;

    this->status_ = Status_s(VOLT_TX_CODE_SPAN, // voltTxCodeSpan
                             TORQ_RX_CODE_SPAN, // torqRxCodeSpan
                             CURR_RATED,        // currRated
                             TORQ_RATED,        // torqRated
                             VOLT_MAX,          // voltMax
                             CURR_MAX,          // currMax
                             TORQ_MAX,          // torqMax
                             TORQ_CONSTANT      // torqConstant
    );

    this->updateMotorMap();
    this->registerRecvCallback();
    this->updateCtrlId();

    LOG::info("GM3510",
              " %s: An instance of GM3510 created, rxBaseId:%hx, txBaseId:%hx",
              this->name_, this->model_.rxBaseId, this->model_.txBaseId);
}

MotorTypeDef_e GM3510::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ != ComType_e::FDCAN &&
        this->comType_ != ComType_e::CAN) {
        rslt |= 1;
        LOG::error("GM3510", " %s: only support FDCAN or CAN comtype",
                   this->name_);
    }

    if (this->workMode_ != WorkMode_e::TRIP_VOLT) {
        rslt |= 1;
        LOG::error("GM3510", " %s: WorkMode is not supported", this->name_);
    }

    if (this->offsetId_ > 3) {
        rslt |= 1;
        LOG::error("GM3510", " %s: Max Offset ID is only 3!", this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        LOG::error("GM3510", " %s: Max TxFreq is only 1000!", this->name_);
    }

    return rslt;
}
