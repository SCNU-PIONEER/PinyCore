#pragma once

#include "FSMState.hpp"
#include "./ArmMotor.hpp"
#include "MsgImpl.hpp"

namespace ARM {

enum class FSMState_e : uint8_t { STOP = 0, NORMAL };

static constexpr float DEFAULT_JOINT_SPEED = 0.8f; // rad/s

enum class JointState_e : uint8_t { FINISH_STATE = 0, MOVING_STATE };

} //namespace ARM

class Arm {
public:
    Arm();

    void update();

    // void setTargetPose(const JointRoute_s _route);

    ARM::JointState_e jointStateFlag = ARM::JointState_e::FINISH_STATE;


    StateFactory<ARM::FSMState_e> stateFactory_;

    ArmMsg_s msg_ = {};

    ARM::Motors motors;

    ARM::Joint target_joints = {};

    // TODO: Cubic polynomial trajectory planning in joint spaces
    // TODO: quintic polynomial trajectory planning in joint spaces
    // TODO: arc trajectory planning in Cartesian space
    // TODO: traight line trajectory planning in Cartesian space
    bool moveJ(const ARM::Joint &_pos);
    bool moveL(const std::array<float, 3> &_xyz, const ARM::Joint &_pose);
    bool moveC(const std::array<float, 3> &_xyz, const ARM::Joint &_pose);

    bool isEnabled = false;

private:
};
