#include "Bipedal.hpp"
#include "Filter.hpp"
#include "Bipedal.hpp"
#include "M3508.hpp"
#include "DM8009.hpp"
#include "sdkconfig.h"
#include "Bsp_can.hpp"

#include "OSnew.hpp"

extern canHandle HCAN1; // wheel
extern canHandle HCAN2; // hip

using namespace CHASSIS::BIPEDAL;
using namespace PINYMOTOR;

Bipedal::Bipedal()
{
    InitConfig_s rM3508Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = 500.f,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = false,
    };
    motor_[0] = new DJIMOTOR::M3508("WheelR", rM3508Config);

    InitConfig_s rfDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 1,
        .txFreq = 500.f,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = false,
    };
    motor_[1] = new DMMOTOR::DM8009("HipRF", rfDM8009Config);

    InitConfig_s rbDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 1,
        .txFreq = 500.f,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = false,
    };
    motor_[2] = new DMMOTOR::DM8009("HipRB", rbDM8009Config);

    InitConfig_s lM3508Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 2,
        .txFreq = 500.f,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = true,
    };
    motor_[3] = new DJIMOTOR::M3508("WheelL", lM3508Config);

    InitConfig_s lfDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 3,
        .txFreq = 500.f,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = true,
    };
    motor_[4] = new DMMOTOR::DM8009("HipLF", lfDM8009Config);

    InitConfig_s lbDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 4,
        .txFreq = 500.f,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = true,
    };
    motor_[5] = new DMMOTOR::DM8009("HipLB", lbDM8009Config);
}

void Bipedal::stop()
{
    for (auto &i : motor_) {
        i->cmd(MotorCmdType_e::OFF);
    }
}

void Bipedal::enter()
{
    for (auto &i : motor_) {
        i->cmd(MotorCmdType_e::ON);
    }
}

void Bipedal::update()
{
    legL.update();
    legR.update();
}
