#include "MsgImpl.hpp"
#include <memory>

#include "./Arm.hpp"
#include "./ArmMotor.hpp"
#include "StmLog.hpp"
#include "ArmNormalState.hpp"
#include "ArmStopState.hpp"
#include "Dwt.hpp"

using namespace ARM;


//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ config ━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// static std::array<float, 6> theta = { 0.0f, M_PI_2, -M_PI_2, 0.0f, -M_PI_2, 0.0f };
// static std::array<float, 6> a = { 0.0f, 5.0f, 5.0f, 0.f, 0.0f, 0.0f };
// static std::array<float, 6> alpha = { M_PI_2, 0.0f, 0.0f, M_PI_2, -M_PI_2, 0.0f };
// static std::array<float, 6> d = { 0.0f, 0.0f, 0.0f, 1.f, 1.f, 1.f };
// static std::array<float, 6> offset = { 0.0f, 0.0f, 0.0f, 0.f, 0.f, 0.f };
//
// static MDH mdh(theta, a, alpha, d, offset);

// TODO: 根据实际机械臂的关节限制设置angleMin和angleMax
static Joint10 angleMin{ { -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f } };
static Joint10 angleMax{ { 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f } };
//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ config ━━━━━━━━━━━━━━━━━━━━━━━━━━━━


Arm::Arm() : motors(angleMin, angleMax)
{
    /* FSM */
    stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(*this));
    stateFactory_.addState(FSMState_e::NORMAL, std::make_unique<NormalState>(*this));
    stateFactory_.init(stateFactory_.getNextState(FSMState_e::STOP));

    LOG::info("ARM", "register");

    motors.stop();
}

void Arm::update(MsgBus_s *_param)
{
    if (xQueueReceive(_param->armQueue, &msg, 0) == pdTRUE) {
    }
    motors.update();
    stateFactory_.update();
}

void Arm::setTargetPose(const JointRoute_s _route) { RouteDta = _route; }

void Arm::moveRoute()
{
    if (moveOneGoal(RouteDta.pose[RouteDta.point_cnt]) == jointStateFlag) {
        if (RouteDta.pose[RouteDta.point_cnt]._delay != 0) {
            LOG::info("ARM", "Start move delay");
            Dwt::instance().delayMs(static_cast<uint32_t>(RouteDta.pose[RouteDta.point_cnt]._delay));
            LOG::info("ARM", "End move delay");
        }
        RouteDta.point_cnt++;
        if (RouteDta.point_cnt == RouteDta.target_point) {
            RouteDta.point_cnt = 0;
            LOG::info("ARM", "Move all point done");
            msg.state = FSMState_e::NORMAL;
        }
    }
}

JointState_e Arm::moveOneGoal(const Joint10D_s &_goal)
{
    if (jointStateFlag == JointState_e::FINISH_STATE) {
        for (int i = 0; i < 10; i++) {
            if (!isWithinRange(_goal.joints[i], motors.jointInfos[i].angle_min, motors.jointInfos[i].angle_max)) {
                LOG::error("ARM", "Joint%d move goal error", i + 1);
                return jointStateFlag;
            }
        }
        target_joints.joints = _goal.joints;
        motors.safety.setJointSpeedLimit(motors.currentJoints, target_joints.joints);
        motors.safety.setAllAngleLimit(target_joints.joints);
        jointStateFlag = JointState_e::MOVING_STATE;
    }

    Joint10 initDeltaJoints = _goal.joints - motors.currentJoints;
    float maxDeltaAngle = AbsMaxOf10(initDeltaJoints);
    if (maxDeltaAngle < 0.02f) {
        jointStateFlag = JointState_e::FINISH_STATE;
        LOG::info("ARM", "Move point%d done", RouteDta.point_cnt);
    }

    motors.ctrl(target_joints.joints);
    return jointStateFlag;
}

// void Arm::updatePose(const ARM::Joint &_joints)
// {
//     auto pose = kinematic.fkine2pose(_joints);
//     this->pose = pose;
// }
//
// void Arm::moveJ(const Joint &_pos) { motors.ctrl(_pos); }
//
// void Arm::moveJBlock(const ARM::Joint &_pos)
// {
//     moveJ(_pos);
//
//     while (!motors.isMoving(_pos)) {
//         vTaskDelay(5);
//     }
// }
//
// void Arm::moveL(const std::array<float, 3> &_xyz, const std::array<float, 3> &_rpy, const ARM::Joint &_pose)
// {
//     const auto thetas = kinematic.ikinePose(_xyz, _rpy, _pose);
//     motors.ctrl(thetas);
// }
//
// void Arm::moveLBlock(const std::array<float, 3> &_xyz, const std::array<float, 3> &_rpy, const ARM::Joint &_pose)
// {
//     moveL(_xyz, _rpy, _pose);
//
//     while (!motors.isMoving(_pose)) {
//         vTaskDelay(5);
//     }
// }
