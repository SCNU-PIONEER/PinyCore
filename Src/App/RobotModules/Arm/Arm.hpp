#pragma once

#include "./ArmPlan.hpp"
#include "FSMState.hpp"
#include "./ArmMotor.hpp"
#include "MsgImpl.hpp"
#include "./ArmKinematic.hpp"
#include "./ArmHand.hpp"
#include "./ArmOreBin.hpp"
#include "./ArmVideo.hpp"

#include "TxPacket.hpp"
#include "PacketImpl.hpp"

namespace ARM {

enum class JointState_e : uint8_t { FINISH_STATE = 0, MOVING_STATE };

class UITxPacket : public COMM::TxPacket<COMM::UIArmMsgPacket> {
public:
    UITxPacket() : COMM::TxPacket<COMM::UIArmMsgPacket>(30.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

} //namespace ARM

class Arm {
public:
    Arm();

    void update(const MsgBus_s *_param);
    void updatePose(const ARM::Joint &_joints);

    void setTargetPose(const ARM::JointRoute_s _route);

    void moveRoute();

    ARM::JointState_e moveOneGoal(const ARM::Joint_s &_goal);

    ARM::JointState_e jointStateFlag = ARM::JointState_e::FINISH_STATE;


    StateFactory<ARM::FSMState_e> stateFactory_;

    ARM::Motors motors;

    ArmMsg_s msg = {};

    ARM::Hand hand;

    ARM::Joint_s target_joints = {};

    ARM::JointRoute_s RouteDta = {};

    ARM::Pose_s pose = {};

    ARM::UITxPacket uiPacket;

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
    ARM::VideoCtrl videoCtrl;

private:
    // ARM::Kinematic kinematic;
};
