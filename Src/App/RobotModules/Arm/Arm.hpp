#pragma once

#include "ArmPlan.hpp"
#include "FSMState.hpp"
#include "./ArmMotor.hpp"
#include "MsgImpl.hpp"
#include "./ArmKinematic.hpp"
#include "./ArmHand.hpp"
#include "./ArmOreBin.hpp"


namespace ARM {
//一键：从矿仓取矿，公路穿越，资源区取矿（一次取两个，针对不同位置有123）
enum class OneClick_e : uint8_t { ORE_BIN_EXTRACT = 0, HIGHWAY_CROSS, ORE_PICK1, ORE_PICK2, ORE_PICK3 };

enum class ArmMode_e : uint8_t { LEFT = 0, RIGHT, TWO };

enum class FSMState_e : uint8_t { STOP = 0, NORMAL, PLAN, TEACH };

enum class JointState_e : uint8_t { FINISH_STATE = 0, MOVING_STATE };

} //namespace ARM

class Arm {
public:
    Arm();

    void update(MsgBus_s *_param);
    void updatePose(const ARM::Joint &_joints);

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

    ARM::Pose_s pose = {};


    // TODO: Cubic polynomial trajectory planning in joint spaces
    // TODO: quintic polynomial trajectory planning in joint spaces
    // TODO: arc trajectory planning in Cartesian space
    // TODO: traight line trajectory planning in Cartesian space
    // void moveJ(const ARM::Joint &_pos);
    // void moveJBlock(const ARM::Joint &_pos);
    // void moveL(const std::array<float, 3> &_xyz, const std::array<float, 3> &_rpy, const ARM::Joint &_pose);
    // void moveLBlock(const std::array<float, 3> &_xyz, const std::array<float, 3> &_rpy, const ARM::Joint &_pose);
    // void moveC(const std::array<float, 3> &_xyz, const ARM::Joint &_pose);

    bool isEnabled = false;

    ARM::OreBin orebin;

private:
    // ARM::Kinematic kinematic;
};
