#include "Arm.hpp"
#include "StmLog.hpp"
#include <memory>
#include "ArmNormalState.hpp"
#include "ArmStopState.hpp"
#include "ArmPlanState.hpp"
#include "ArmTeachState.hpp"
#include "Dwt.h"

using namespace ARM;

Arm::Arm() : safety(motors)
{
    msg_ = armMsg{};
    /* FSM */
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::STOP),
                           std::make_unique<StopState>(*this));
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::RUN),
                           std::make_unique<ArmNormalState>(*this));
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::PLAN),
                           std::make_unique<PlanState>(*this));
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::TEACH),
                           std::make_unique<ArmTeachState>(*this));
    LOG::info("ARM", "register");
}

void Arm::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->armQueue), &msg_, 0) == pdTRUE) {
    };
    motors.update();
    stateFactory_.update();
}

void Arm::moveOneJoint(Joint7D _target_joints)
{
    //UT缓启动
    if (!safety.speedLimit(1)) {
        LOG::Logger::instance().error(LOCATION, "ARM", "Speed limit exceeded");
        return;
    }
    if (!safety.angleLimit(target_joints)) {
        LOG::Logger::instance().error(LOCATION, "ARM", "Angle limit exceeded");
        return;
    }
    //output
    motors.ctrl(target_joints);
}

void Arm::setTargetPose(const JointRoute_s _route)
{
    target_pose = _route.pose;
    target_point = _route.point;
    // pump = _route.pump;
}

void Arm::moveRoute()
{
    if (motors.moveOneGoal(target_pose[point_cnt]) ==
        motors.jointStateFlag) { //bug
        /*为了到达某点后停止一段时间*/
        if (target_pose[point_cnt].delay != 0) {
            log.error(LOCATION, "ARM", "Start move delay");
            dwt_delay_ms(target_pose[point_cnt].delay);
            log.error(LOCATION, "ARM", "End move delay");
        }
        point_cnt++;
        if (point_cnt == target_point) {
            point_cnt = 0;
            log.info(LOCATION, "ARM", "Move all point done");
            //change state to normal
        }
    }
}

void Arm::teach()
{
    //     Joint7D goal;
    // for (uint8_t i = 0; i < 7; i++) {
    //     goal.j[i] = _tp_data.joint[i];
    // }

    // /*checkout first*/
    //     if (!motors.checkGoal(goal)) {
    //         log.error(LOCATION, "Teach", "Teach goal out of range");
    //         return;
    //     }

    //     /*arm reset, can't be interrupt*/
    //     if (mmove_one_goal(goal) == FINISH_STATE) {
    //         teach_tag = Arm::TEACHED;
    //     } else {
    //         teach_tag == Arm::LAUNCHING;
    //     }


    // /*control,can be interrupt*/
    //     if (_tp_data.push == 1) {
    //         pumpCtrl.apply(PUMPCONFIGS::ALL_PUMP_ON);
    //     } else {
    //         pumpCtrl.apply(PUMPCONFIGS::ALL_PUMP_OFF);
    //     }


    //     for (uint8_t i = 0; i < 7; i++) {
    //         target_joints.j[i] = _tp_data.joint[i];
    //     }
    //     safety.speedLimit(0.5);
    //     safety.angleLimit(target_joints);
    //     motors.ctrl(target_joints); //output
}