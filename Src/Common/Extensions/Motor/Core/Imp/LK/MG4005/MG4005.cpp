#include "MG4005.hpp"
#include "StmLog.hpp"
#include <cstring>

using namespace PINYMOTOR;
using namespace LKMOTOR;

/**
 * @brief 构造函数
 */
MG4005::MG4005(const char _name[16], InitConfig_s _config, WorkMode_e _workmode)
        : LKMotor4005(_name, _config, _workmode)
{
    this->status_ = Status_s(POWER_MAX,      // PMax
                             TORQ_MAX,       // TMax
                             SPEED_MAX,      // speedMax
                             SPEED_CONSTANT, // speedConstant
                             TORQ_CONSTANT,  // torqueConstant
                             CURR_MAX,       // CurrMax
                             INNER_REDUCTION_RATIO);

    this->data_.zeroAng = 0.0f;
    this->data_.angLast = 0.0f;
    this->data_.multipCirAng = 0.0f;
    this->regInfo_.model.reductionRatio = INNER_REDUCTION_RATIO;
    this->regInfo_.model.measureMax = ENCODER_SPAN;
    this->regInfo_.model.measureMin = 0;

    LOG::info("LKMOTOR", "%s: An instance of MG4005 created", this->regInfo_.name);
}
