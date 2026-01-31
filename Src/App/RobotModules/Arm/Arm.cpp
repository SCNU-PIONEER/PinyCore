#include "MsgImpl.hpp"
#include <memory>

#include "./Arm.hpp"
#include "./ArmMotor.hpp"
#include "StmLog.hpp"
#include "ArmNormalState.hpp"
#include "ArmStopState.hpp"
#include "Dwt.hpp"

using namespace ARM;

Arm::Arm()
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
        uint32_t i = 0;
    }
    motors.update();
    stateFactory_.update();
}

void Arm::setTargetPose(const JointRoute_s _route) { RouteDta = _route; }
void Arm::moveRoute()
{
    if (moveOneGoal(RouteDta.pose[RouteDta.point_cnt]) == jointStateFlag) {
        /*为了到达某点后停止一段时间*/
        if (RouteDta.pose[RouteDta.point_cnt]._delay != 0) {
            LOG::info("ARM", "Start move delay");
            Dwt::instance().delayMs(static_cast<uint32_t>(RouteDta.pose[RouteDta.point_cnt]._delay));
            LOG::info("ARM", "End move delay");
        }
        RouteDta.point_cnt++;
        if (RouteDta.point_cnt == RouteDta.target_point) {
            RouteDta.point_cnt = 0;
            LOG::info("ARM", "Move all point done");
            //change state to normal
            msg.state = FSMState_e::NORMAL;
        }
    }
}

JointState_e Arm::moveOneGoal(const Joint10D_s &_goal)
{
    /*第一次进入之后*/
    if (jointStateFlag == JointState_e::MOVING_STATE) {
        rateCnt++;
        float rate = static_cast<float>(rateCnt) / 2000.0f;
    }
    /*第一次进入*/
    if (jointStateFlag == JointState_e::FINISH_STATE) {
        /*joint1 - joint10*/
        for (int i = 0; i < 10; i++) {
            if (!IS_WITHIN_RANGE(_goal.joints[i], motors.jointInfos[i].angle_min, motors.jointInfos[i].angle_max)) {
                LOG::error("ARM", "Joint%d move goal error", i + 1);
                return jointStateFlag;
            }
        }
        /*init target joints*/
        for (uint8_t i = 0; i < 10; i++) {
            target_joints.joints[i] = _goal.joints[i];
        }

        Joint10 initDeltaJoints = _goal.joints - motors.currentJoints;
        float initMaxDeltaAngle = AbsMaxOf10(initDeltaJoints);

        float maxTime = initMaxDeltaAngle / DEFAULT_JOINT_SPEED;
        motors.safety.setJointSpeedLimit(maxTime, initDeltaJoints);
        motors.safety.setAllAngleLimit(target_joints.joints);

        jointStateFlag = JointState_e::MOVING_STATE;
    }
    Joint10 initDeltaJoints = _goal.joints - motors.currentJoints;
    float maxDeltaAngle = AbsMaxOf10(initDeltaJoints);
    if (maxDeltaAngle < 0.02f) {
        rateCnt = 0;
        jointStateFlag = JointState_e::FINISH_STATE;
        LOG::info("ARM", "Move point%d done", RouteDta.point_cnt);
    }

    // output
    motors.ctrl(target_joints.joints);

    return jointStateFlag;
}
