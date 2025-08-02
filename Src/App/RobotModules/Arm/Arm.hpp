/*
 * @File         : Arm.hpp
 * @Brief        : 
 * @Version      : 
 * @Author       : 3687402504@qq.com
 * @LastEditTime : 2025-08-02 17:30:14
 * Copyright 2025 by SCNU-PIONEER (c), All Rights Reserved.
 */

/*
* @file Arm.hpp
* @brief Arm class definition for controlling a robotic arm.
*/

#pragma once

#include "ARMSafety.hpp"
#include "FSMState.hpp"
#include "ArmMotor.hpp"
#include "MsgImpl.hpp"
#include "Pump.hpp"
#include "StmLog.hpp"


namespace ARM {
enum class FSMState_e : uint8_t { STOP = 0, RUN, PLAN, TEACH };

}

class Arm {
public:
    Arm();

    void update(void *_param);
    void moveOneJoint(Joint7D _target_joints);
    
    void setTargetPose(const JointRoute_s _route);
    void moveOneGoal(const Joint7D& _goal);
    void moveRoute();
    
    void teach();

    LOG::Logger &log = LOG::Logger::instance();
    
    ARM::Safety safety;
    StateFactory stateFactory_;
    armMsg msg_;
    ARM::Motors motors;
    

    const Joint7D *target_pose = nullptr;
    uint8_t target_point = 0;
    uint8_t point_cnt = 0;
    const Pump_s *pump;
    Joint7D target_joints;
};
