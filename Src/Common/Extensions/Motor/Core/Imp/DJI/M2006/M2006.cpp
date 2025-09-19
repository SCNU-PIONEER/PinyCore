#include "M2006.hpp"

#include "DJIMotor.hpp"

#include "StmLog.hpp"

#include <cstring>

using namespace PINYMOTOR;
using namespace DJIMOTOR;
M2006::M2006(const char _name[16], InitConfig_s _config)
        : DJIMotor(_name, _config)
{
    LOG::CHECK(checkBaseConfig());

    regInfo_.model.measureMax = 8191;
    regInfo_.model.measureMin = 0;
    regInfo_.model.reductionRatio = 36.f;
    regInfo_.model.rxBaseId = 0x200;

    if (_config.offsetId > 3)
        regInfo_.model.txBaseId = 0x1FF;
    else
        regInfo_.model.txBaseId = 0x200;

    this->status_ =
            Status_s(VOLT_TX_CODE_SPAN, // voltTxCodeSpan
                     CURR_TX_CODE_SPAN, // currTxCodeSpan
                     CURR_RX_CODE_SPAN, // currRxCodeSpan
                     VOLT_MAX,          //voltmax
                     //搭配c610无堵转电流和堵转扭矩数据，所以还是用额定数据
                     CURR_MAX, // currMax
                     TORQ_MAX, // torqMax
                     KN        // Kn
            );

    this->updateMotorMap();
    this->registerRecvCallback(regInfo_.model.rxBaseId + regInfo_.offsetId);
    this->updateCtrlMode();

    LOG::info(
            "M2006",
            " %s: An instance of M2006 created, rxBaseId:0x%hx, txBaseId:0x%hx",
            regInfo_.name, regInfo_.model.rxBaseId, regInfo_.model.txBaseId);
}

MotorTypeDef_e M2006::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (regInfo_.comType != ComType_e ::CAN) {
        rslt |= 1;
        LOG::error("M2006", " %s: only support CAN comtype", regInfo_.name);
    }

    if (regInfo_.workMode != WorkMode_e::QUAD_CURR) {
        rslt |= 1;
        LOG::error("M2006", " %s: WorkMode is not supported", regInfo_.name);
    }

    if (regInfo_.offsetId > 8) {
        rslt |= 1;
        LOG::error("M2006", " %s: Max Offset ID is only 8!",

                   regInfo_.name);
    }

    if (AUX_.txFreq > 1000) {
        rslt |= 1;
        LOG::error("M2006", " %s: Max TxFreq is only 1000!", regInfo_.name);
    }

    return rslt;
}
