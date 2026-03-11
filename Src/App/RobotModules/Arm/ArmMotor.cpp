#include "ArmMotor.hpp"
#include "Soc.hpp"
#include "DM8009.hpp"
#include "DM4340.hpp"
#include "DM3507.hpp"
#include "DM10010L.hpp"
#include "MG8016E_i6_RS485.hpp"


using namespace ARM;

Motors::Motors(Joint10 _angleMin, Joint10 _angleMax) : safety(*this, _angleMin, _angleMax)
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
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
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
        .workMode = WorkMode_e::PDESVDES,
        .offsetId = 5,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };
    InitConfig_s dm3507Config6 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::PDESVDES,
        .offsetId = 6,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };
    InitConfig_s dm8009Config7 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN3),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::PDESVDES,
        .offsetId = 7,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };
    InitConfig_s dm4340Config8 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN3),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::PDESVDES,
        .offsetId = 8,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };
    InitConfig_s dm3507Config9 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN3),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::PDESVDES,
        .offsetId = 3,
        .txFreq = 500.0f,
        .posPID = nullptr,
        .velPID = nullptr,
    };
    InitConfig_s dm3507Config10 = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN3),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::PDESVDES,
        .offsetId = 4,
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
    motors.all_motors[6] = new DMMOTOR::DM8009("joint7", dm8009Config7);
    motors.all_motors[7] = new DMMOTOR::DM4340("joint8", dm4340Config8);
    motors.all_motors[8] = new DMMOTOR::DM3507("joint9", dm3507Config9);
    motors.all_motors[9] = new DMMOTOR::DM3507("joint10", dm3507Config10);
};

void Motors::update()
{
    size_t index = 0;
    for (auto &motor : motors.all_motors) {
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
bool Motors::init() { return true; }

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

bool Motors::isMoving(const Joint10 &_target)
{
    Joint10 deltaJoints = _target - currentJoints;
    float maxDeltaAngle = AbsMaxOf10(deltaJoints);
    return maxDeltaAngle < 0.02f;
}

void Motors::ctrl(const Joint10 &_targetJoints)
{
    Joint10 temp = _targetJoints;

    safety.setAllAngleLimit(temp);

    size_t index = 0;
    for (const auto &motor : motors.all_motors) {
        motor->cmdPos(temp[index]);
        index++;
    }
}

void Motors::handYaw(Joint10 &_msgTargetJoints)
{
    _msgTargetJoints[4] = _msgTargetJoints[4];
    _msgTargetJoints[5] = -_msgTargetJoints[4];
    _msgTargetJoints[9] = _msgTargetJoints[8];
    _msgTargetJoints[10] = -_msgTargetJoints[8];
}

void Motors::handPitch(Joint10 &_msgTargetJoints)
{
    _msgTargetJoints[4] = _msgTargetJoints[5];
    _msgTargetJoints[5] = _msgTargetJoints[5];
    _msgTargetJoints[9] = _msgTargetJoints[9];
    _msgTargetJoints[10] = _msgTargetJoints[9];
}
