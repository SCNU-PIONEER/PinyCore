#include "Arm.hpp"
#include "ArmMotor.hpp"
#include "StmLog.hpp"
#include <memory>
#include "ArmNormalState.hpp"
#include "ArmStopState.hpp"
#include "ArmPlanState.hpp"
#include "ArmTeachState.hpp"
#include "Dwt.h"
#include "MotorCommonMacros.hpp"

using namespace ARM;

Arm::Arm() : safety(motors)
{
    msg_ = armMsg{};
    /* FSM */
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::STOP),
                           std::make_unique<StopState>(*this));
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::NORMAL),
                           std::make_unique<ArmNormalState>(*this));
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::PLAN),
                           std::make_unique<PlanState>(*this));
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::TEACH),
                           std::make_unique<ArmTeachState>(*this));
    stateFactory_.init(stateFactory_.getNextState(
            static_cast<uint8_t>(FSMState_e::STOP))); //new add
    LOG::info("ARM", "register");
}

void Arm::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->armQueue), &msg_, 0) == pdTRUE) {
    };
    motors.update();
    stateFactory_.update();
}

void Arm::setTargetPose(const JointRoute_s _route)
{
    RouteDta.target_pose = _route.pose;
    RouteDta.target_point = _route.point;
    RouteDta.pump = _route.pump;
}

void Arm::moveRoute()
{
    if (moveOneGoal(RouteDta.target_pose[RouteDta.point_cnt]) ==
        jointStateFlag) { //bug
        /*为了到达某点后停止一段时间*/
        if (RouteDta.target_pose[RouteDta.point_cnt].delay != 0) {
            log.error(LOCATION, "ARM", "Start move delay");
            dwt_delay_ms(RouteDta.target_pose[RouteDta.point_cnt].delay);
            log.error(LOCATION, "ARM", "End move delay");
        }
        RouteDta.point_cnt++;
        if (RouteDta.point_cnt == RouteDta.target_point) {
            RouteDta.point_cnt = 0;
            log.info(LOCATION, "ARM", "Move all point done");
            //change state to normal
            msg_.state = State_e::NORMAL;
        }
    }
}

Arm::JointState_e Arm::moveOneGoal(const Joint7D &_goal)
{
    Joint7D deltaJoints = _goal - motors.current_joints;
    float maxDeltaAngle = AbsMaxOf7(deltaJoints);

    /*第一次进入之后*/
    if (jointStateFlag == JointState_e::MOVING_STATE) {
        RouteDta.rateCnt++;
        float rate = static_cast<float>(RouteDta.rateCnt) / 2000.0f;
        target_joints.j[0] =
                PINYMOTOR::s_curve_acc(target_joints.j[0], _goal.j[0], 35, 2);
        target_joints.j[6] =
                PINYMOTOR::s_curve_acc(target_joints.j[6], _goal.j[6], 30, 3);
    }
    /*第一次进入*/
    if (jointStateFlag == JointState_e::FINISH_STATE) {
        /*joint1 - joint2*/
        for (int i = 0; i < 2; i++) {
            if (!IS_WITHIN_RANGE(_goal.j[i], motors.jointInfos[i].angle_min,
                                 motors.jointInfos[i].angle_max)) {
                return jointStateFlag;
                log.error(LOCATION, "ARM", "Joint%d move goal error", i + 1);
            }
        }

        /*joint3*/
        float highTemp = motors.joint3HighPoint(_goal.j[1]);
        float lowTemp = motors.joint3LowPoint(_goal.j[1]);
        if (!IS_WITHIN_RANGE(_goal.j[2], highTemp, lowTemp)) {
            log.error(LOCATION, "ARM", "Joint3 move goal error");
            return jointStateFlag;
        }

        /*joint4 - joint7*/
        for (int i = 3; i < 7; i++) {
            if (!IS_WITHIN_RANGE(_goal.j[i], motors.jointInfos[i].angle_min,
                                 motors.jointInfos[i].angle_max)) {
                log.error(LOCATION, "ARM", "Joint%d move goal error", i + 1);
                return jointStateFlag;
            }
        }

        // 初始化目标关节值（除1和6外）
        for (uint8_t i = 1; i < 6; i++) {
            target_joints.j[i] = _goal.j[i];
        }

        target_joints.j[0] = motors.current_joints.j[0];
        target_joints.j[6] = motors.current_joints.j[6];

        float maxTime = maxDeltaAngle / DEFAULT_JOINT_SPEED;
        safety.setJointSpeedLimit(maxTime, deltaJoints);
        safety.setAllAngleLimit(target_joints);

        pumpCtrl.apply(&RouteDta.pump[RouteDta.point_cnt]);

        jointStateFlag = JointState_e::MOVING_STATE;
    }

    if (maxDeltaAngle < 0.02f) {
        RouteDta.rateCnt = 0;
        jointStateFlag = JointState_e::FINISH_STATE;
        log.info(LOCATION, "ARM", "Move point%d done", RouteDta.point_cnt);
    }

    // output
    motors.ctrl(target_joints);

    return jointStateFlag;
}
