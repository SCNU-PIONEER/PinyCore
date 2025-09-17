#include "DM4310.hpp"

#include "DMMotor.hpp"

#include "StmLog.hpp"

#include <cstring>

using namespace PINYMOTOR;
using namespace DMMOTOR;
DM4310::DM4310(const char _name[16], InitConfig_s _config)
        : DMMotor(_name, _config)
{
    LOG::CHECK(checkBaseConfig());

    regInfo_.model.measureMax = 16383;
    regInfo_.model.measureMin = 0;
    regInfo_.model.reductionRatio = 1.f;
    regInfo_.model.rxBaseId = 0x10;
    regInfo_.model.txBaseId = 0x00;

    this->status_ = Status_s(P_MAX,             // PMax
                             V_MAX,             // VMax
                             T_MAX,             // TMax
                             MIT_KP_MAX,        // MITKpMax
                             MIT_KP_MAX,        // MITKdMax
                             CURR_TX_CODE_SPAN, // currTxCodeSpan
                             CURR_RATED,        // currRated
                             TORQ_RATED,        // torqRated
                             CURR_MAX,          // currMax
                             TORQ_MAX,          // torqMax
                             TORQ_CONSTANT      // torqConstant
    );

    this->registerRecvCallback();
    this->updateCtrlId();

    LOG::info(
            "DM4310",
            " %s: An instance of DM4310 created, rxBaseId:0x%hx, txBaseId:0x%hx",
            regInfo_.name, regInfo_.model.rxBaseId, regInfo_.model.txBaseId);
}

MotorTypeDef_e DM4310::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (regInfo_.comType != ComType_e::FDCAN &&
        regInfo_.comType != ComType_e::CAN) {
        rslt |= 1;
        LOG::error("DM4310", " %s: only support FDCAN or CAN comtype",
                   regInfo_.name);
    }

    if (regInfo_.workMode == WorkMode_e::QUAD_VOLT) {
        rslt |= 1;
        LOG::error("DM4310", " %s: WorkMode is not supported", regInfo_.name);
    }

    if (regInfo_.offsetId > 9) {
        rslt |= 1;
        LOG::error("DM4310", " %s: Max Offset ID is only 9!", regInfo_.name);
    }

    if (AUX_.txFreq > 1000) {
        rslt |= 1;
        LOG::error("DM4310", " %s: Max TxFreq is only 1000!", regInfo_.name);
    }

    return rslt;
}
