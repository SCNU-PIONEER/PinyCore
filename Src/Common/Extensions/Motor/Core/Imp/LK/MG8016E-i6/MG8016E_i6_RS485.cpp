#include "MG8016E_i6_RS485.hpp"
#include "LKMotorRS485.hpp"
#include "StmLog.hpp"
using namespace PINYMOTOR;
using namespace LKMOTOR;

MG8016Ei6RS485::MG8016Ei6RS485(const char _name[16], InitConfig_s _config, WorkMode_e _mode)
        : LKMotorRS485(_name, _config, _mode)
{
    this->regInfo_.model.measureMax = ENCODER_SPAN;
    this->regInfo_.model.measureMin = 0;

    this->status_ = Status_s(POWER_MAX,      // PMax
                             TORQ_MAX,       // TMax
                             SPEED_MAX,      // speedMax
                             SPEED_CONSTANT, // speedConstant
                             TORQ_CONSTANT,  // torqueConstant
                             TX_CURR_MAX,    // txCurrMax
                             INNER_REDUCTION_RATIO);

    this->data_.zeroAng = 0.0f;
    this->data_.angLast = 0.0f;
    this->data_.multipCirAng = 0.0f;
    regInfo_.model.reductionRatio = 1.f;
    regInfo_.model.rxBaseId = 0;
    regInfo_.model.txBaseId = 0;

    LOG::info("LKMOTOR", "%s: An instance of MG8016_Ei6_RS485 created", this->regInfo_.name);
}
