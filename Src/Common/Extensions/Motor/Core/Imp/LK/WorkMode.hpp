#include "IMotor.hpp"

namespace PINYMOTOR::LKMOTOR {

enum class WorkMode_e : MotorTypeDef_e {
    UNKNOWN,
    QUAD_CURR,
    VOLT,
    VDES,
    MIT_TT,
    SINGLE_PDES,
    SINGLE_PDESVDES,
    MULTI_PDESVDES,
    MULTI_PDES,
    PDESVDES,
    INC_PDES,
    INC_PDESVDES,
};

} // namespace PINYMOTOR::LKMOTOR
