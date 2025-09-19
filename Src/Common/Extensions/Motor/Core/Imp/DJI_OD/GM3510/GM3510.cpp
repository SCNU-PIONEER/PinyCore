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

    regInfo_.model.measureMax = 8191;
    regInfo_.model.measureMin = 0;
    regInfo_.model.reductionRatio = 1.f;
    regInfo_.model.rxBaseId = 0x204;

    regInfo_.model.txBaseId = 0x1FF;

    this->status_ = Status_s(VOLT_TX_CODE_SPAN, // voltTxCodeSpan
                             TORQ_RX_CODE_SPAN, // torqRxCodeSpan
                             VOLT_MAX,          // voltMax
                             CURR_MAX,          // currMax
                             TORQ_MAX,          // torqMax
                             KN                 // Kn
    );

    this->updateMotorMap();
    this->registerRecvCallback();
    this->updateCtrlMode();

    LOG::info(
            "GM3510",
            " %s: An instance of GM3510 created, rxBaseId:0x%hx, txBaseId:0x%hx",
            regInfo_.name, regInfo_.model.rxBaseId, regInfo_.model.txBaseId);
}

MotorTypeDef_e GM3510::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (regInfo_.comType != ComType_e::FDCAN &&
        regInfo_.comType != ComType_e::CAN) {
        rslt |= 1;
        LOG::error("GM3510", " %s: only support FDCAN or CAN comtype",
                   regInfo_.name);
    }

    if (regInfo_.workMode != WorkMode_e::TRIP_VOLT) {
        rslt |= 1;
        LOG::error("GM3510", " %s: WorkMode is not supported", regInfo_.name);
    }

    if (regInfo_.offsetId > 3) {
        rslt |= 1;
        LOG::error("GM3510", " %s: Max Offset ID is only 3!", regInfo_.name);
    }

    if (AUX_.txFreq > 1000) {
        rslt |= 1;
        LOG::error("GM3510", " %s: Max TxFreq is only 1000!", regInfo_.name);
    }

    return rslt;
}
