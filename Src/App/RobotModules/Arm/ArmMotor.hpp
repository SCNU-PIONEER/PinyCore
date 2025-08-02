#pragma once
#include "IMotor.hpp"
#include "ARMSafety.hpp"
#include "StmLog.hpp"


namespace ARM {

class Motors {
public:
    Motors();
    void init();
    void update();
    void stop();
    void ctrl(const Joint7D &_target_joints);

    LOG::Logger &log = LOG::Logger::instance();
    enum class Moveresult_e : std::uint8_t { FINISHED, MOVING, OUTOFRANGE };
    Moveresult_e moveOneGoal(const Joint7D& _goal);
    struct JointInfo_s {
    float angle_min;
    float angle_max;
};
    JointInfo_s jointInfos[7];

protected:
    void biasJoint3Angle();
    float joint3HighPoint(float _target);
    float joint3LowPoint(float _target);

private:

    std::unique_ptr<PINYMOTOR::IMotor> motor[7];
    Joint7D current_joints;
};

}// namespace ARM
