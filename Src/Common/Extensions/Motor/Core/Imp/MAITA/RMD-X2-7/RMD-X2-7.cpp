#include "./RMD-X2-7.hpp"
#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace MTMOTOR;

static constexpr char TAG[] = "RMDX27";

RMDX27::RMDX27(const char _name[16], InitConfig_s _config)
        : MTMotor(_name, _config)
{
    LOG::CHECK(checkBaseConfig());

    regInfo_.model.measureMax = 65535;
    regInfo_.model.measureMin = 0;
    regInfo_.model.reductionRatio = 1.f;
    regInfo_.model.rxBaseId = 0x240;
    regInfo_.model.txBaseId = 0x140;

    this->status_ = Status_s(SPEED_MAX, //speedMax
                             CURR_MAX,  // currMax
                             TORQ_MAX,  // torqMax
                             NP,        // np
                             INTER_RR,  // interRr
                             KN         // Kn
    );

    this->registerMotor();
    this->registerRecvCallback(regInfo_.model.rxBaseId + regInfo_.offsetId);
    updateCtrlMode();

    LOG::info(TAG, " %s: created, rxBaseId:0x%hx, txBaseId:0x%hx",
              regInfo_.name, regInfo_.model.rxBaseId, regInfo_.model.txBaseId);
}

MotorTypeDef_e RMDX27::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (regInfo_.comType != ComType_e::CAN) {
        rslt |= 1;
        LOG::error(TAG, " %s: only support CAN comtype", regInfo_.name);
    }

    if (regInfo_.workMode != WorkMode_e::PDESVDES) {
        rslt |= 1;
        LOG::error(TAG, " %s: only support PDESVDES WorkMode", regInfo_.name);
    }

    if (regInfo_.offsetId > 32) {
        rslt |= 1;
        LOG::error(TAG, " %s: Max Offset ID is only 32!", regInfo_.name);
    }

    if (AUX_.txFreq > 1000) {
        rslt |= 1;
        LOG::error(TAG, " %s: Max TxFreq is only 1000!", regInfo_.name);
    }

    return rslt;
}
