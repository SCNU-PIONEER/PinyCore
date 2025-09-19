#include "DM3519.hpp"

#include "DMMotor.hpp"

#include "StmLog.hpp"

#include <cstring>

using namespace PINYMOTOR;
using namespace DMMOTOR;
DM3519::DM3519(const char _name[16], InitConfig_s _config)
        : DMMotor(_name, _config)
{
    LOG::CHECK(checkBaseConfig());

    regInfo_.model.measureMax = 16383;
    regInfo_.model.measureMin = 0;
    regInfo_.model.reductionRatio = (3591.f / 187.f);
    regInfo_.model.rxBaseId = 0x10;
    regInfo_.model.txBaseId = 0x00;

    this->status_ = Status_s(P_MAX,             // PMax
                             V_MAX,             // VMax
                             T_MAX,             // TMax
                             MIT_KP_MAX,        // MITKpMax
                             MIT_KP_MAX,        // MITKdMax
                             CURR_TX_CODE_SPAN, // currTxCodeSpan
                             CURR_MAX,          // currMax
                             TORQ_MAX,          // torqMax
                             KN                 // Kn
    );

    this->registerRecvCallback(regInfo_.model.rxBaseId + regInfo_.offsetId);
    this->updateCtrlMode();

    LOG::info(
            "DM3519",
            " %s: An instance of DM3519 created, rxBaseId:0x%hx, txBaseId:0x%hx",
            regInfo_.name, regInfo_.model.rxBaseId, regInfo_.model.txBaseId);
}

MotorTypeDef_e DM3519::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (regInfo_.comType != ComType_e::FDCAN &&
        regInfo_.comType != ComType_e::CAN) {
        rslt |= 1;
        LOG::error("DM3519", " %s: only support FDCAN or CAN comtype",
                   regInfo_.name);
    }
    if (regInfo_.workMode == WorkMode_e::QUAD_VOLT) {
        rslt |= 1;
        LOG::error("DM3519", " %s: WorkMode is not supported", regInfo_.name);
    }

    if (regInfo_.offsetId > 9) {
        rslt |= 1;
        LOG::error("DM3519", " %s: Max Offset ID is only 9!", regInfo_.name);
    }

    if (AUX_.txFreq > 1000) {
        rslt |= 1;
        LOG::error("DM3519", " %s: Max TxFreq is only 1000!", regInfo_.name);
    }

    return rslt;
}
