#include "ArmMotor.hpp"
#include "ArmKinematic.hpp"
#include "Soc.hpp"
#include "DM8009.hpp"
#include "DM4340.hpp"
#include "DM3507.hpp"
#include "DM10010L.hpp"
#include "MG8016E_i6_RS485.hpp"
#include "StmLog.hpp"
#include <cstdint>


using namespace ARM;

Motors::Motors(Joint _angleMin, Joint _angleMax) : safety(*this, _angleMin, _angleMax)
{
    InitConfig_s mg8016Config1 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&huart3),
        .comType = ComType_e::RS485,
        .workMode = WorkMode_e::MULTI_PDESVDES,
        .offsetId = 1,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };
    InitConfig_s dm10010Config2 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN3),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::PDESVDES,
        .offsetId = 2,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };
    InitConfig_s dm8009Config3 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::PDESVDES,
        .offsetId = 3,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };
    InitConfig_s dm4340Config4 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::PDESVDES,
        .offsetId = 4,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };
    InitConfig_s dm3507Config5 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::EMIT,
        .offsetId = 5,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };
    InitConfig_s dm3507Config6 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::EMIT,
        .offsetId = 6,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };


    motors.all_motors[0] = new LKMOTOR::MG8016Ei6RS485("joint1", mg8016Config1);
    motors.all_motors[1] = new DMMOTOR::DM10010L("joint2", dm10010Config2);
    motors.all_motors[2] = new DMMOTOR::DM8009("joint3", dm8009Config3);
    motors.all_motors[3] = new DMMOTOR::DM4340("joint4", dm4340Config4);
    motors.all_motors[4] = new DMMOTOR::DM3507("joint5", dm3507Config5);
    motors.all_motors[5] = new DMMOTOR::DM3507("joint6", dm3507Config6);
};

uint16_t id = 0;
void Motors::update()
{
    size_t index = 0;
    for (auto &motor : motors.all_motors) {
        // 温度报警，防止堵转过热
        if (motor->data().tempture > 50.f) {
            LOG::error("MOTORS", "%u, temp: %.2f C", index, motor->data().tempture);
            // BUZZER::Buzzer::getInstance().playDJI();
            id = index;
        }
        // if (motor->globalState == PINYMOTOR::GlobalState_e::OFFLINE) {
        //     LOG::error("MOTORS", "Motor %u, offline", index + 1);
        // }

        /* singleCirAng range: 0 ~ 2PI, we need range: -pi ~ pi */
        if (motor->data().singleCirAng >= std::numbers::pi_v<float>) {
            currentJoints[index] = motor->data().singleCirAng - (2 * std::numbers::pi_v<float>);
        } else {
            currentJoints[index] = motor->data().singleCirAng;
        }
        index++;
    }
}

// TODO: 电机复位
bool Motors::init()
{
    if ((motors.dmMotor2->errorCode_ != PINYMOTOR::DMMOTOR::ErrorCode_e::MOTOR_DISABLE) &&
        (motors.dmMotor2->errorCode_ != PINYMOTOR::DMMOTOR::ErrorCode_e::MOTOR_ENABLE)) {
        motors.dmMotor2->clearError();
    }
    if (motors.dmMotor5->errorCode_ == PINYMOTOR::DMMOTOR::ErrorCode_e::OVER_CURRENT ||
        motors.dmMotor5->errorCode_ == PINYMOTOR::DMMOTOR::ErrorCode_e::ROTOR_OVER_HEAT) {
        motors.dmMotor5->clearError();
    }
    if (motors.dmMotor6->errorCode_ == PINYMOTOR::DMMOTOR::ErrorCode_e::OVER_CURRENT ||
        motors.dmMotor5->errorCode_ == PINYMOTOR::DMMOTOR::ErrorCode_e::ROTOR_OVER_HEAT) {
        motors.dmMotor6->clearError();
    }
    // motors.dmMotor5->setPosZero();
    // motors.dmMotor6->setPosZero();
    return true;
}

bool Motors::homing(const Joint _target_joints)
{
    if (isWithinRange(_target_joints, safety.getAngleMax(), safety.getAngleMin())) {
        uint8_t index = 0;                //BUG:
        for (uint8_t i = 0; i < 6; ++i) { //右臂
            if (_target_joints[i] - currentJoints[i] < 0.05) {
                index += 1;
            }
        }
        if (index == 6) {
            return true;
        }


        safety.setSpeed(0.1);
        ctrl(_target_joints);
    }
    return false;
}

void Motors::stop()
{
    for (const auto &motor : motors.all_motors) {
        motor->cmd(MotorCmdType_e::OFF);
    }
}

void Motors::enable()
{
    for (const auto &motor : motors.all_motors) {
        motor->cmd(MotorCmdType_e::ON);
    }
}

bool Motors::isMoving(const Joint &_target)
{
    Joint deltaJoints = _target - currentJoints;
    float maxDeltaAngle = Kinematic::absMax(deltaJoints);
    return maxDeltaAngle < 0.02f;
}

void Motors::ctrl(const Joint &_targetJoints)
{
    Joint temp = _targetJoints;

    safety.setAllAngleLimit(temp);

    for (uint8_t i = 0; i < 6; ++i) {
        if (i < 4) {
            motors.all_motors[i]->cmdPosVel(temp[i], ref_speed._[i]);
        } else {
            motors.all_motors[i]->cmdMIT(temp[i], ref_speed._[i], RATED_TORQUE);
        }
    }
}