#pragma once

#include "FSMState.hpp"
#include "./ArmMotor.hpp"
#include "MsgImpl.hpp"

#include "./ArmHand.hpp"
#include "M3508.hpp"
#include "Bsp_pwm.hpp"

namespace ARM {

enum class FSMState_e : uint8_t { STOP = 0, NORMAL };

static constexpr float DEFAULT_JOINT_SPEED = 0.8f; // rad/s

enum class JointState_e : uint8_t { FINISH_STATE = 0, MOVING_STATE };

class OreBin {
public:
    OreBin() = default;

    void init();
    void loop(ArmMsg_s &_msg);
    void stop();

    uint16_t release_angle = 1000;
    uint16_t stuck_angle = 2000;

private:
    std::unique_ptr<Pwm> pwm1 = nullptr;
    std::unique_ptr<Pwm> pwm2 = nullptr;
};

class Lifting {
public:
    Lifting();
    void lift();
    void stop();
    void enable();

private:
    PINYMOTOR::DJIMOTOR::M3508 motor;
};

} //namespace ARM

class Arm {
public:
    Arm();

    void update(MsgBus_s *_param);

    // void setTargetPose(const JointRoute_s _route);

    ARM::JointState_e jointStateFlag = ARM::JointState_e::FINISH_STATE;


    StateFactory<ARM::FSMState_e> stateFactory_;

    ARM::Motors motors;

    ArmMsg_s msg = {};

    ARM::Hand hand;

    ARM::Joint target_joints = {};

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
};
