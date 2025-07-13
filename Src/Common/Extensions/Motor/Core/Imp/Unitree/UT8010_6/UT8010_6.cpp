#include "UT8010_6.hpp"

using namespace PINYMOTOR;
using namespace UTMOTOR;

UT8010_6::UT8010_6(const char _name[16], InitConfig_s _config,
                   DMA_HandleTypeDef *_dmaHandle)
        : UTMotor(_name, _config, _dmaHandle)
{
    strcpy(this->model_.name, "UT8010_6");
    this->model_.measureMax = 0;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = RR;
    this->model_.rxBaseId = 0;
    this->model_.txBaseId = 0;
    this->ctrlId_ = _config.offsetId;

    this->status_ = Status_s{ P_MAX,    V_MAX,    T_MAX,     KP_MAX, KD_MAX,
                              TRQE_MAX, CURR_MAX, SPEED_MAX, Kn };

    this->registerMotor();
    this->registerRecvCallback();

    checkBaseConfig();

    LOG::info("UT8010_6", " %s: An instance of UT8010_6 created, ctrlId:%hx",
              this->name_, this->ctrlId_);
}

MotorTypeDef_e UT8010_6::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ != PINYMOTOR::ComType_e ::RS485) {
        rslt |= 1;
        LOG::error("UT8010_6", " %s: only support RS485 comtype", this->name_);
    }

    if (this->workMode_ != WorkMode_e::EMIT) {
        rslt |= 1;
        LOG::error("UT8010_6", " %s: WorkMode only support EMIT", this->name_);
    }

    if (this->offsetId_ > 15) {
        rslt |= 1;
        LOG::error("UT8010_6", " %s: Max Offset ID is only 15!", this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        LOG::error("UT8010_6", " %s: Max TxFreq is only 1000!", this->name_);
    }

    return rslt;
}
