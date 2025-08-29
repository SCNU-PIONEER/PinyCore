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
#include <cstdint>

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

    InitConfig_s ut80106Config = { .pComHandle = reinterpret_cast<uint32_t *>(
                                           &UNITREE_UART),
                                   .comType = PINYMOTOR::ComType_e::RS485,
                                   .workMode = PINYMOTOR::WorkMode_e::EMIT,
                                   .offsetId = static_cast<uint8_t>(1),
                                   .txFreq = 500.0f,
                                   .posPID = nullptr,
                                   .velPID = nullptr,
                                   .torqPID = nullptr };

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

    InitConfig_s gmConfig = { .pComHandle =
                                      reinterpret_cast<uint32_t *>(&HCAN3),
                              .comType = PINYMOTOR::ComType_e::FDCAN,
                              .workMode = PINYMOTOR::WorkMode_e::QUAD_CURR,
                              .offsetId = static_cast<uint8_t>(7),
                              .txFreq = 500.0f,
                              .posPID = nullptr,
                              .velPID = nullptr,
                              .torqPID = nullptr };

    motors.utMotor = new UTMOTOR::UT80106("joint1", std::move(ut80106Config),
                                          &UNITREE_DMA);
    motors.dmMotor1 = new DMMOTOR::DM8009("joint2", std::move(dmJointConf));
    motors.dmMotor2 = new DMMOTOR::DM8009("joint3", std::move(dmJointConf));
    motors.dmMotor3 = new DMMOTOR::DM4310("joint4", std::move(dmJointConf));
    motors.dmMotor4 = new DMMOTOR::DM4310("joint5", std::move(dmJointConf));
    motors.dmMotor5 = new DMMOTOR::DM4310("joint6", std::move(dmJointConf));
    motors.djMotor = new DJIMOTOR::GM6020("joint7", std::move(gmConfig));
}

void Motors::init()
{
    homingUT();
    //TODO:check motor offline
    //enable all motor
    for (int i = 0; i < 7; i++) {
        motors.all_motors[i]->cmd(MotorCmdType_e::ON);
    }
}

void Motors::update()
{
    // 更新所有电机状态
    for (int i = 0; i < 7; i++) {
        current_joints.j[i] = motors.all_motors[i]->data().singleCirAng;
    }

    /*平行四边形关系 - 确保关节3角度在安全范围内*/
    biasJoint3Angle();
}

void Motors::stop()
{
    for (int i = 0; i < 7; i++) {
        motors.all_motors[i]->cmd(MotorCmdType_e::OFF);
    }
}

void Motors::ctrl(const Joint7D &_target_joints)
{
    // 依次发送目标角度and speed到每个关节电机
    for (int i = 1; i < 7; i++) {
        motors.all_motors[i]->cmd(MotorCmdType_e::SET_POS, _target_joints.j[i]);
        motors.all_motors[i]->cmd(MotorCmdType_e::SET_VEL, ref_speed._[i]);
    }
    motors.utMotor->cmd(MotorCmdType_e::SET_POS,
                        _target_joints.j[0] + unitreeAngleFix);
    motors.utMotor->cmd(MotorCmdType_e::SET_VEL, ref_speed.joint1);
}


void Motors::biasJoint3Angle()
{
    jointInfos[2].angle_min =
            joint3HighPoint(motors.utMotor->data().singleCirAng);
    jointInfos[2].angle_max =
            joint3LowPoint(motors.utMotor->data().singleCirAng);
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

    motors.utMotor->setKd(0.03f);
    ref_speed.joint1 = 2.0f; // target speed

    if (fabs(motors.utMotor->data().torq) >= 0.37f &&
        fabs(motors.utMotor->data().spdRadps) < 0.1f) {
        unitreeAngleFix = motors.utMotor->data().multipCirAng;

        motors.utMotor->setKp(1.0f);
        motors.utMotor->setKd(0.f);
        ref_speed.joint1 = 0; //target speed

        motors.utMotor->setZeroAng();
        current_joints.j[0] = 0;
        utResetState = true; // 标记为已完成
        return true;
    } else {
        motors.utMotor->cmd(MotorCmdType_e::SET_VEL, ref_speed.joint1);
        motors.utMotor->cmd(MotorCmdType_e::SET_POS, 0.f); //output targetJiont
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

void Motors::setUTsmoothStart()
{
    float currentKp = motors.utMotor->getKp();
    float currentKd = motors.utMotor->getKd();

    float newKp = s_curve_acc(currentKp, UNITREE_KP, 30, 0);
    float newKd = s_curve_acc(currentKd, UNITREE_KD, 30, 1);

    motors.utMotor->setKp(newKp);
    motors.utMotor->setKd(newKd);
}
