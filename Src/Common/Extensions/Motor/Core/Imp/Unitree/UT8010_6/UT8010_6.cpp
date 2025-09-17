#include "UT8010_6.hpp"
#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace UTMOTOR;

UT80106::UT80106(const char _name[16], InitConfig_s _config,
                 DMA_HandleTypeDef *_dmaHandle)
        : UTMotor(_name, _config, _dmaHandle)
{
    LOG::CHECK(checkBaseConfig());

    regInfo_.model.measureMax = 0;
    regInfo_.model.measureMin = 0;
    regInfo_.model.reductionRatio = RR;
    regInfo_.model.rxBaseId = 0;
    regInfo_.model.txBaseId = 0;
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

    this->registerRecvCallback();


    LOG::info("UT8010_6", " %s: An instance of UT8010_6 created, ctrlId:0x%hx",
              regInfo_.name, this->ctrlId_);
}

MotorTypeDef_e UT80106::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (regInfo_.comType != PINYMOTOR::ComType_e ::RS485) {
        rslt |= 1;
        LOG::error("UT8010_6", " %s: only support RS485 comtype",
                   regInfo_.name);
    }

    if (regInfo_.workMode != WorkMode_e::EMIT) {
        rslt |= 1;
        LOG::error("UT8010_6", " %s: WorkMode only support EMIT",
                   regInfo_.name);
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
