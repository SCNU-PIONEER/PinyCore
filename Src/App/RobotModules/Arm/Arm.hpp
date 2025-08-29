#pragma once

#include "ARMSafety.hpp"
#include "FSMState.hpp"
#include "ArmMotor.hpp"
#include "MsgImpl.hpp"
#include "Pump.hpp"
#include "StmLog.hpp"


namespace ARM {
enum class FSMState_e : uint8_t { STOP = 0, NORMAL, PLAN, TEACH };

}

class Arm {
    static constexpr float DEFAULT_JOINT_SPEED = 0.8f; // rad/s
    struct RouteData_s {
        const Joint7D *target_pose = nullptr;
        uint8_t target_point = 0;
        uint8_t point_cnt = 0;
        int16_t rateCnt = 0;
        const PUMP::State_e *pump;
    }; //only used in plan state,need to arrange

public:
    Arm();

    void update(void *_param);

    void setTargetPose(const JointRoute_s _route);

    void moveRoute();

    enum class JointState_e : uint8_t { FINISH_STATE = 0, MOVING_STATE };
    JointState_e jointStateFlag = JointState_e::FINISH_STATE;
    JointState_e moveOneGoal(const Joint7D &_goal);

    LOG::Logger &log = LOG::Logger::instance();
    PUMP::Controller &pumpCtrl = PUMP::Controller::instance();


    ARM::Safety safety;
    StateFactory stateFactory_;
    armMsg msg_;
    ARM::Motors motors;

    Joint7D target_joints;


private:
    RouteData_s RouteDta;
};
