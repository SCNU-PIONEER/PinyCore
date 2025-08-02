/*
 * @File         : ArmMotor.cpp
 * @Brief        : 
 * @Version      : 
 * @Author       : 3687402504@qq.com
 * @LastEditTime : 2025-08-02 17:21:06
 * Copyright 2025 by SCNU-PIONEER (c), All Rights Reserved.
 */
/*
 * @File         : ArmMotor.cpp
 * @Brief        : 
 * @Version      : 
 * @Author       : 3687402504@qq.com
 * @LastEditTime : 2025-08-02 17:13:30
 * Copyright 2025 by SCNU-PIONEER (c), All Rights Reserved.
 */

#include "stm32h7xx_hal.h"
#include "arm_math_types.h"
#include "sdkconfig.h"
#include "ArmMotor.hpp"
#include "DM4310.hpp"
#include "DM8009.hpp"
#include "UT8010_6.hpp"
#include "Bsp_can.hpp"
#include "GM6020.hpp"

extern canHandle HCAN2;
extern canHandle HCAN3;

using namespace PINYMOTOR;
using namespace ARM;

Motors::Motors()
{
    jointInfos[0] = { .angle_min = -2.90f, .angle_max = 0.f};
    jointInfos[1] = { .angle_min = 0.f, .angle_max = 1.17f};
    jointInfos[2] = { .angle_min = -0.87f, .angle_max = -0.10f};
    jointInfos[3] = { .angle_min = -1.43f, .angle_max = 0.99f};
    jointInfos[4] = { .angle_min = -3.14f, .angle_max = 1.34f};
    jointInfos[5] = { .angle_min = -2.20f, .angle_max = 2.17f};
    jointInfos[6] = { .angle_min = -6.14f, .angle_max = 6.14f};

    

    InitConfig_s jointConf = { .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
                               .comType = PINYMOTOR::ComType_e::FDCAN,
                               .workMode = PINYMOTOR::WorkMode_e::PDESVDES,
                               .offsetId = static_cast<uint8_t>(4),
                               .txFreq = 500.0f,
                               .posPID = nullptr,
                               .velPID = nullptr,
                               .torqPID = nullptr };
    InitConfig_s ut80106Config = { .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2), //huart2
                               .comType = PINYMOTOR::ComType_e::RS485,
                               .workMode = PINYMOTOR::WorkMode_e::PDESVDES,
                               .offsetId = static_cast<uint8_t>(4),
                               .txFreq = 500.0f,
                               .posPID = nullptr,
                               .velPID = nullptr,
                               .torqPID = nullptr };
    // DMA_HandleTypeDef hdma_usart2_rx;

    // motor[0] = std::unique_ptr<IMotor>(
    //         new UTMOTOR::UT80106("joint1", std::move(ut80106Config), &hdma_usart2_rx));
    motor[1] = std::unique_ptr<IMotor>(
            new DMMOTOR::DM8009("joint2", std::move(jointConf)));
    motor[2] = std::unique_ptr<IMotor>(
            new DMMOTOR::DM8009("joint3", std::move(jointConf)));
    motor[3] = std::unique_ptr<IMotor>(
            new DMMOTOR::DM4310("joint4", std::move(jointConf)));
    motor[4] = std::unique_ptr<IMotor>(
            new DMMOTOR::DM4310("joint5", std::move(jointConf)));
    motor[5] = std::unique_ptr<IMotor>(
            new DMMOTOR::DM4310("joint6", std::move(jointConf)));
    motor[6] = std::unique_ptr<IMotor>(
            new DJIMOTOR::GM6020("joint7", std::move(jointConf)));
    

}

void Motors::init() {

}

void Motors::update()
{
    // 更新所有电机状态
    for (uint8_t i = 0; i < 7; i++) {
        current_joints.j[i] = motor[i]->data().singleCirAng;
    }
    /*平行四边形关系 - 确保关节3角度在安全范围内*/
    biasJoint3Angle();
}

void Motors::stop() {
    for (uint8_t i = 0; i < 7; i++) {
        motor[i]->cmd(MotorCmdType_e::OFF);
    }
}

void Motors::ctrl(const Joint7D &_target_joints)
{
    // 依次发送目标角度到每个关节电机
    for (uint8_t i = 0; i < 7; ++i) {
        motor[i]->cmd(MotorCmdType_e::SET_POS, _target_joints.j[i]);
    }
}


Motors::Moveresult_e Motors::moveOneGoal(const Joint7D& goal)
{
    // 1. 检查所有关节目标是否在限制范围
    for (int i = 0; i < 7; ++i) {
        float min = jointInfos[i].angle_min;
        float max = jointInfos[i].angle_max;
        if (goal.j[i] < min || goal.j[i] > max) {
            log.error(LOCATION, "ARM", "Joint%d goal out of range", i + 1);
            return Moveresult_e::OUTOFRANGE;
        }
    }
    // 特殊关节3限制
    float highTemp = joint3HighPoint(goal.j[1]);
    float lowTemp  = joint3LowPoint(goal.j[1]);
    if (goal.j[2] < highTemp || goal.j[2] > lowTemp) {
        log.error(LOCATION, "ARM", "Joint3 goal out of range");
        return Moveresult_e::OUTOFRANGE;
    }

    // 2. 计算关节差值
    Joint7D deltaJoints = goal - current_joints;
    float maxDeltaAngle = AbsMaxOf7(deltaJoints);

    // 3. 插值生成目标并输出
    Joint7D interpJoints;
    for (int i = 0; i < 7; ++i) {
        // interp_joints.j[i] = s_curve_acc(current_joints.j[i], goal.j[i], 35, i);
    }
    ctrl(interpJoints);

    // 4. 判断是否到达
    if (maxDeltaAngle < 0.02f) {
        LOG::Logger::instance().info(LOCATION, "ARM", "Move goal done");
        return Moveresult_e::FINISHED;
    }
    return Moveresult_e::MOVING;
}

void Motors::biasJoint3Angle() {
    jointInfos[2].angle_min = joint3HighPoint(motor[1]->data().singleCirAng);
    jointInfos[2].angle_max = joint3LowPoint(motor[1]->data().singleCirAng);
}

float Motors::joint3HighPoint(float _target) {
    return ((-0.8119f) * _target) - 1.05f;
}

float Motors::joint3LowPoint(float _target) {
    if (_target > 0.68f) {
        return ((-1.08163f) * _target) + 0.7355f;
    } else {
        return 0.0f;
    }
}

bool Safety::angleLimit(const Joint7D &_joints)
{
    for (int i = 0; i < 7; ++i) {
        float min = motors->jointInfos[i].angle_min;
        float max = motors->jointInfos[i].angle_max;  
        if (_joints.j[i] < min || _joints.j[i] > max) {
            return false;
        }
    }
    return true;
}

bool Safety::speedLimit(const float _speed)
{
    //only limit
    return (_speed >= DEFAULT_JOINT_SPEED_MIN && _speed <= DEFAULT_JOINT_SPEED_MAX);

}

