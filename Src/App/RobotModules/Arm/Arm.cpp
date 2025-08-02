/*
 * @File         : Arm.cpp
 * @Brief        : 
 * @Version      : 
 * @Author       : 3687402504@qq.com
 * @LastEditTime : 2025-08-02 15:53:51
 * Copyright 2025 by SCNU-PIONEER (c), All Rights Reserved.
 */

#include "Arm.hpp"
#include "StmLog.hpp"
#include <memory>
#include "ArmNormalState.hpp"
#include "ArmStopState.hpp"
#include "ArmPlanState.hpp"
#include "ArmTeachState.hpp"
#include "Dwt.h"

using namespace ARM;

Arm::Arm()
:safety(&motors)
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
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg_, 0) ==
        pdTRUE) {
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
    pump = _route.pump;
}



void Arm::moveRoute()
{
    // 路径点已全部完成
    if (point_cnt >= target_point) {
        point_cnt = 0;
        LOG::Logger::instance().info(LOCATION, "ARM", "All route points finished");
        // 切回正常状态
        return;
    }
    // 移动到当前路径点
    auto result = motors.moveOneGoal(target_pose[point_cnt]);
    if (result == ARM::Motors::Moveresult_e::FINISHED) {
        if (target_pose[point_cnt].delay > 0) {
            LOG::Logger::instance().info(LOCATION, "ARM", "Start move delay");
            dwt_delay_ms(target_pose[point_cnt].delay);
            LOG::Logger::instance().info(LOCATION, "ARM", "Delay finished");
        }
        point_cnt++;
    } else if (result == ARM::Motors::Moveresult_e::OUTOFRANGE) {
        LOG::Logger::instance().error(LOCATION, "ARM", "Move goal out of range");
        // 切回正常状态
        point_cnt = 0;
    }
}

void Arm::teach()
{
    
}