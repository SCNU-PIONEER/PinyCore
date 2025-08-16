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

#include "PidBasic.hpp"
#include "Projdefs.hpp"


#include "sdkconfig.h"
#include "ArmMotor.hpp"
#include "DM4310.hpp"
#include "DM8009.hpp"
#include "UT8010_6.hpp"

#include "GM6020.hpp"
#include "MotorCommonMacros.hpp"

#include "UTMotorMsg.hpp"

extern canHandle HCAN2;
extern canHandle HCAN3;
extern UART_HandleTypeDef UNITREE_UART;
extern DMA_HandleTypeDef UNITREE_DMA;

using namespace PINYMOTOR;
using namespace ARM;

Motors::Motors()
{
    jointInfos[0] = { .angle_min = -2.90f, .angle_max = 0.f };
    jointInfos[1] = { .angle_min = 0.f, .angle_max = 1.17f };
    jointInfos[2] = { .angle_min = -0.87f, .angle_max = -0.10f };
    jointInfos[3] = { .angle_min = -1.43f, .angle_max = 0.99f };
    jointInfos[4] = { .angle_min = -3.14f, .angle_max = 1.34f };
    jointInfos[5] = { .angle_min = -2.20f, .angle_max = 2.17f };
    jointInfos[6] = { .angle_min = -6.14f, .angle_max = 6.14f };


    InitConfig_s dmJointConf = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = PINYMOTOR::ComType_e::FDCAN,
        .workMode = PINYMOTOR::WorkMode_e::PDESVDES,
        .offsetId = static_cast<uint8_t>(4),
        .txFreq = 500.0f,
        .posPID = std::unique_ptr<PID>(
                new positonalPid(100.f, 0.0f, 100.f, 0.f, 0.f, 200.f, 0.01f)),
        .velPID = std::unique_ptr<PID>(
                new positonalPid(0.06f, 0.005f, 0.0f, 0.0f, 2.f, 25.2f, 0.1f)),
        .torqPID = nullptr
    };
    InitConfig_s ut80106Config = { .pComHandle = reinterpret_cast<uint32_t *>(
                                           &UNITREE_UART),
                                   .comType = PINYMOTOR::ComType_e::RS485,
                                   .workMode = PINYMOTOR::WorkMode_e::PDESVDES,
                                   .offsetId = static_cast<uint8_t>(4),
                                   .txFreq = 500.0f,
                                   .posPID = nullptr,
                                   .velPID = nullptr,
                                   .torqPID = nullptr };
    InitConfig_s gmConfig = { .pComHandle =
                                      reinterpret_cast<uint32_t *>(&HCAN3),
                              .comType = PINYMOTOR::ComType_e::FDCAN,
                              .workMode = PINYMOTOR::WorkMode_e::PDESVDES,
                              .offsetId = static_cast<uint8_t>(4),
                              .txFreq = 500.0f,
                              .posPID = nullptr,
                              .velPID = nullptr,
                              .torqPID = nullptr };

    // motor[0] = std::unique_ptr<IMotor>(new UTMOTOR::UT80106(
    //         "joint1", std::move(ut80106Config), &UNITREE_DMA));    //public
    motor[1] = std::unique_ptr<IMotor>(
            new DMMOTOR::DM8009("joint2", std::move(dmJointConf)));
    motor[2] = std::unique_ptr<IMotor>(
            new DMMOTOR::DM8009("joint3", std::move(dmJointConf)));
    motor[3] = std::unique_ptr<IMotor>(
            new DMMOTOR::DM4310("joint4", std::move(dmJointConf)));
    motor[4] = std::unique_ptr<IMotor>(
            new DMMOTOR::DM4310("joint5", std::move(dmJointConf)));
    motor[5] = std::unique_ptr<IMotor>(
            new DMMOTOR::DM4310("joint6", std::move(dmJointConf)));
    motor[6] = std::unique_ptr<IMotor>(
            new DJIMOTOR::GM6020("joint7", std::move(gmConfig)));
}

void Motors::init()
{
    //TODO:check motor offline
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

void Motors::stop()
{
    for (uint8_t i = 0; i < 7; i++) {
        motor[i]->cmd(MotorCmdType_e::OFF);
    }
}

void Motors::ctrl(const Joint7D &_target_joints)
{
    // 依次发送目标角度and speed到每个关节电机
    for (uint8_t i = 0; i < 7; ++i) {
        motor[i]->cmd(MotorCmdType_e::SET_POS, _target_joints.j[i]);
        motor[i]->cmd(MotorCmdType_e::SET_VEL, ref_speed._[i]);
    }
}

Motors::JointState_e Motors::moveOneGoal(const Joint7D &_goal)
{
    Joint7D deltaJoints = _goal - current_joints;
    float maxDeltaAngle = AbsMaxOf7(deltaJoints);

    /*第一次进入之后*/
    if (jointStateFlag == JointState_e::MOVING_STATE) {
        motionState.rateCnt++;
        float rate = static_cast<float>(motionState.rateCnt) / 2000.0f;
        motionState.targetJoints.j[0] =
                s_curve_acc(motionState.targetJoints.j[0], _goal.j[0], 35,
                            2); // 保持原有S曲线逻辑（待修复BUG）
        motionState.targetJoints.j[6] =
                s_curve_acc(motionState.targetJoints.j[6], _goal.j[6], 30,
                            3); // 保持原有S曲线逻辑（待修复BUG）

        /* joint1 */
        motor[0]->cmd(MotorCmdType_e::SET_POS,
                      motionState.targetJoints.j[0] +
                              motionState.unitreeAngleFix);
        /* joint7 */
        motor[6]->cmd(MotorCmdType_e::SET_POS, motionState.targetJoints.j[6]);
    }

    /*第一次进入*/
    if (jointStateFlag == JointState_e::FINISH_STATE) {
        /*joint1 - joint2*/
        for (int i = 0; i < 2; i++) {
            if (!IS_WITHIN_RANGE(_goal.j[i], jointInfos[i].angle_min,
                                 jointInfos[i].angle_max)) {
                return jointStateFlag;
                log.error(LOCATION, "ARM", "Joint%d move goal error", i + 1);
            }
        }

        /*joint3*/
        float highTemp = joint3HighPoint(_goal.j[1]);
        float lowTemp = joint3LowPoint(_goal.j[1]);
        if (!IS_WITHIN_RANGE(_goal.j[2], highTemp, lowTemp)) {
            log.error(LOCATION, "ARM", "Joint3 move goal error");
            return jointStateFlag;
        }

        /*joint4 - joint7*/
        for (int i = 3; i < 7; i++) {
            if (!IS_WITHIN_RANGE(_goal.j[i], jointInfos[i].angle_min,
                                 jointInfos[i].angle_max)) {
                log.error(LOCATION, "ARM", "Joint%d move goal error", i + 1);
                return jointStateFlag;
            }
        }

        // 初始化目标关节值（除1和6外）
        for (uint8_t i = 1; i < 6; i++) {
            motionState.targetJoints.j[i] = _goal.j[i];
        }

        motionState.unitreeInitAngle = current_joints.j[0];
        motionState.targetJoints.j[0] = current_joints.j[0];
        motionState.targetJoints.j[6] = current_joints.j[6];

        float maxTime = maxDeltaAngle / motionState.jointSpeed;
        // sasetJointSpeedLimit(maxTime, deltaJoints);
        // set_all_angle_limit();


        //pump

        jointStateFlag = JointState_e::MOVING_STATE;
    }

    if (maxDeltaAngle < 0.02f) {
        motionState.rateCnt = 0;
        jointStateFlag = JointState_e::FINISH_STATE;
        log.info(LOCATION, "ARM", "Move point%d done", motionState.pointCnt);
    }

    // 输出所有电机指令（保持原有逻辑）
    // set_all_motor_output();

    return jointStateFlag;
}


void Motors::biasJoint3Angle()
{
    jointInfos[2].angle_min = joint3HighPoint(motor[1]->data().singleCirAng);
    jointInfos[2].angle_max = joint3LowPoint(motor[1]->data().singleCirAng);
}

float Motors::joint3HighPoint(float _target)
{
    return ((-0.8119f) * _target) - 1.05f;
}

float Motors::joint3LowPoint(float _target)
{
    if (_target > 0.68f) {
        return ((-1.08163f) * _target) + 0.7355f;
    } else {
        return 0.0f;
    }
}

bool Motors::homingUT()
{
    static bool utResetState = false; // 新增：记录回零是否完成
    if (utResetState)
        return true; // 已完成则直接返回

    PINYMOTOR::UTMOTOR::TransmitMsg_s utTxMsg;
    memset(&utTxMsg, 0, sizeof(PINYMOTOR::UTMOTOR::TransmitMsg_s));

    // motor[0]->setKd(0.03f);  //not sure
    ref_speed.joint1 = 2.0f; // target speed

    if (fabs(motor[0]->data().torq) >= 0.37f &&
        fabs(motor[0]->data().spdRadps) < 0.1f) {
        float unitreeAngleFix = motor[0]->data().singleCirAng;

        // motor[0]->setKp(1.0f);
        // motor[0]->setKd(0.f);
        ref_speed.joint1 = 0; //target speed

        motor[0]->setZeroAng();
        target_joints.j[0] = 0;
        current_joints.j[0] = 0;
        utResetState = true; // 标记为已完成
        return true;
    } else {
        motor[0]->cmd(MotorCmdType_e::SET_VEL, ref_speed.joint1);
        return false;
        //ooutput
    }
}

bool Motors::checkGoal(Joint7D _goal)
{
    for (int i = 0; i < 2; i++) {
        if (!IS_WITHIN_RANGE(_goal.j[i], jointInfos[i].angle_min,
                             jointInfos[i].angle_max)) {
            log.error(LOCATION, "ARM", "Joint%d move goal error", i + 1);
            return false;
        }
    }
    /*joint3*/
    float highTemp = joint3HighPoint(_goal.j[1]);
    float lowTemp = joint3LowPoint(_goal.j[1]);
    if (!IS_WITHIN_RANGE(_goal.j[2], highTemp, lowTemp)) {
        log.error(LOCATION, "ARM", "Joint3 move goal error");
        return false;
    }
    /*joint4 - joint7*/
    for (int i = 3; i < 7; i++) {
        if (!IS_WITHIN_RANGE(_goal.j[i], jointInfos[i].angle_min,
                             jointInfos[i].angle_max)) {
            log.error(LOCATION, "ARM", "Joint%d move goal error", i + 1);
            return false;
        }
    }
    return true;
} //checkGoal
