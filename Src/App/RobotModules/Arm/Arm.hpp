#pragma once

#include "ArmPlan.hpp"
#include "FSMState.hpp"
#include "./ArmMotor.hpp"
#include "MsgImpl.hpp"
#include "./ArmKinematic.hpp"
#include "./ArmHand.hpp"
#include "./ArmLifting.hpp"
#include "./ArmOreBin.hpp"

#define IS_WITHIN_RANGE(x, M1, M2)                      \
    ({                                                  \
        typeof(x) _x = (x);                             \
        typeof(M1) _m1 = (M1);                          \
        typeof(M2) _m2 = (M2);                          \
        typeof(_m1) _min_val = (_m1 < _m2) ? _m1 : _m2; \
        typeof(_m2) _max_val = (_m1 < _m2) ? _m2 : _m1; \
        (_x >= _min_val && _x <= _max_val);             \
    })

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
    ARM::Lifting lifting;

private:
    int16_t rateCnt = 0;

private:
    // ARM::Kinematic kinematic;
};
