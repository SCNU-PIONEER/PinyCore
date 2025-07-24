#include "UT8010_6.hpp"

using namespace PINYMOTOR;
using namespace UTMOTOR;

UT80106::UT80106(const char _name[16], InitConfig_s _config,
                 DMA_HandleTypeDef *_dmaHandle)
        : UTMotor(_name, std::move(_config), _dmaHandle)
{
    LOG::CHECK(checkBaseConfig());

    strcpy(this->model_.name, "UT8010_6");
    this->model_.measureMax = 0;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = RR;
    this->model_.rxBaseId = 0;
    this->model_.txBaseId = 0;
    this->ctrlId_ = _config.offsetId;

    this->status_ = Status_s{ .PMax = P_MAX,
                              .VMax = V_MAX,
                              .TMax = T_MAX,
                              .KpMax = KP_MAX,
                              .KdMax = KD_MAX,
                              .currMax = CURR_MAX,
                              .torqMax = TRQE_MAX,
                              .speedMax = SPEED_MAX,
                              .Kn = KN };

    this->registerMotor();
    this->registerRecvCallback();


    LOG::info("UT8010_6", " %s: An instance of UT8010_6 created, ctrlId:%hx",
              this->name_, this->ctrlId_);
}

MotorTypeDef_e UT80106::checkBaseConfig()
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
