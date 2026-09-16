#include "MF9025.hpp"
#include "StmLog.hpp"
#include <cstring>

using namespace PINYMOTOR;
using namespace LKMOTOR;

MF9025::MF9025(const char _name[16], InitConfig_s _config, WorkMode_e _workmode)
        : LKMotor9025(_name, _config, _workmode)
{
    this->status_ = Status_s(POWER_MAX,      // PMax
                             TORQ_MAX,       // TMax
                             SPEED_MAX,      // speedMax
                             SPEED_CONSTANT, // speedConstant
                             TORQ_CONSTANT,  // torqueConstant
                             CURR_MAX,       // CurrMax
                             INNER_REDUCTION_RATIO);

    this->regInfo_.model.reductionRatio = INNER_REDUCTION_RATIO;
    this->regInfo_.model.measureMax = ENCODER_SPAN;
    this->regInfo_.model.measureMin = 0;
    LOG::info("LKMOTOR", "%s: An instance of MF9025 created", this->regInfo_.name);
}
