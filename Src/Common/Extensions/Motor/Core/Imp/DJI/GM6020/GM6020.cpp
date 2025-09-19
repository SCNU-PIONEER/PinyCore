#include "GM6020.hpp"

#include "DJIMotor.hpp"

#include "StmLog.hpp"

#include <cstring>

using namespace PINYMOTOR;
using namespace DJIMOTOR;
GM6020::GM6020(const char _name[16], InitConfig_s _config)
        : DJIMotor(_name, _config)
{
    LOG::CHECK(checkBaseConfig());

    regInfo_.model.measureMax = 8191;
    regInfo_.model.measureMin = 0;
    regInfo_.model.reductionRatio = 1.f;
    regInfo_.model.rxBaseId = 0x204;

    if (regInfo_.workMode == WorkMode_e::QUAD_CURR) {
        if (_config.offsetId > 4)
            regInfo_.model.txBaseId = 0x2FE;
        else
            regInfo_.model.txBaseId = 0x1FE;
    } else if (regInfo_.workMode == WorkMode_e::QUAD_VOLT) {
        if (_config.offsetId > 4)
            regInfo_.model.txBaseId = 0x2FF;
        else
            regInfo_.model.txBaseId = 0x1FF;
    }

    this->status_ = Status_s(VOLT_TX_CODE_SPAN, // voltTxCodeSpan
                             CURR_TX_CODE_SPAN, // currTxCodeSpan
                             CURR_RX_CODE_SPAN, // currRxCodeSpan
                             VOLT_MAX,          // voltMax
                             CURR_MAX,          // currMax
                             TORQ_MAX,          // torqMax
                             KN                 // Kn
    );

    this->updateMotorMap();
    this->registerRecvCallback();
    this->updateCtrlId();

    LOG::info(
            "GM6020",
            " %s: An instance of GM6020 created, rxBaseId:0x%hx, txBaseId:0x%hx",
            regInfo_.name, regInfo_.model.rxBaseId, regInfo_.model.txBaseId);
}

MotorTypeDef_e GM6020::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (regInfo_.comType != PINYMOTOR::ComType_e ::CAN) {
        rslt |= 1;
        LOG::error("GM6020", " %s: only support CAN comtype", regInfo_.name);
    }

    if (regInfo_.workMode != WorkMode_e::QUAD_CURR &&
        regInfo_.workMode != WorkMode_e::QUAD_VOLT) {
        rslt |= 1;
        LOG::error("GM6020", "%s: WorkMode is not supported", regInfo_.name);
    }

    if (regInfo_.offsetId > 7) {
        rslt |= 1;
        LOG::error("GM6020", "%s: Max Offset ID is only 7!", regInfo_.name);
    }

    if (AUX_.txFreq > 1000) {
        rslt |= 1;
        LOG::error("GM6020", " %s: Max TxFreq is only 1000!", regInfo_.name);
    }

    return rslt;
}
