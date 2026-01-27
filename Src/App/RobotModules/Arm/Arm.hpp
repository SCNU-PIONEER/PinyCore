#pragma once

#include "ArmPlan.hpp"
#include "FSMState.hpp"
#include "./ArmMotor.hpp"
#include "MsgImpl.hpp"

#include "./ArmHand.hpp"
#include "./ArmLifting.hpp"
#include "./ArmOreBin.hpp"
#include "Dwt.hpp"


namespace ARM {
enum class OneClick_e : uint8_t { ORE_BIN_EXTRACT = 0, HIGHWAY_CROSS, ORE_PICK1, ORE_PICK2, ORE_PICK3 };

enum class ArmMode_e : uint8_t { LEFT = 0, RIGHT, TWO };

enum class FSMState_e : uint8_t { STOP = 0, NORMAL, PLAN, TEACH };

static constexpr float DEFAULT_JOINT_SPEED = 0.8f; // rad/s

enum class JointState_e : uint8_t { FINISH_STATE = 0, MOVING_STATE };

} //namespace ARM

class Arm {
public:
    Arm();

    void update(MsgBus_s *_param);

    void setTargetPose(const ARM::JointRoute_s _route);

    void moveRoute();

    ARM::JointState_e moveOneGoal(const ARM::Joint10D_s &_goal);

    ARM::JointState_e jointStateFlag = ARM::JointState_e::FINISH_STATE;


    StateFactory<ARM::FSMState_e> stateFactory_;

    ARM::Motors motors;

    ArmMsg_s msg = {};

    ARM::Hand hand;

    ARM::Joint10D_s target_joints = {};

    ARM::JointRoute_s RouteDta = {};

    // TODO: Cubic polynomial trajectory planning in joint spaces
    // TODO: quintic polynomial trajectory planning in joint spaces
    // TODO: arc trajectory planning in Cartesian space
    // TODO: traight line trajectory planning in Cartesian space
    bool moveJ(const ARM::Joint &_pos);
    bool moveL(const std::array<float, 3> &_xyz, const ARM::Joint &_pose);
    bool moveC(const std::array<float, 3> &_xyz, const ARM::Joint &_pose);

    bool isEnabled = false;

    ARM::OreBin orebin;
    ARM::Lifting lifting;

private:
    Dwt dwt = Dwt::instance();
    int16_t rateCnt = 0;
};
