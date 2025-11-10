#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"
#include "Dwt.hpp"

namespace ARM {

enum class FSMState_e : uint8_t { STOP = 0, NORMAL, PLAN, TEACH };

struct Msg_s : public Msg {
    FSMState_e state;
};

static constexpr float DEFAULT_JOINT_SPEED = 0.8f; // rad/s

enum class JointState_e : uint8_t { FINISH_STATE = 0, MOVING_STATE };

struct RouteData_s {
    uint8_t target_point = 0;
    uint8_t point_cnt = 0;
    int16_t rateCnt = 0;
}; //only used in plan state,need to arrange

} //namespace ARM

class Arm {
public:
    Arm();

    void update();

    // void setTargetPose(const JointRoute_s _route);

    void moveRoute();

    ARM::JointState_e jointStateFlag = ARM::JointState_e::FINISH_STATE;

    // ARM::JointState_e moveOneGoal(const Joint7D &_goal);

    StateFactory<ARM::FSMState_e> stateFactory_;

    ARM::Msg_s msg_ = {};

    ARM::Msg_s tpmsg_ = {};

    // ARM::Motors motors;

    // Joint7D target_joints = {};

    bool isEnabled = false;

private:
    ARM::RouteData_s RouteDta = {};
    Dwt dwt = Dwt::instance();
};
