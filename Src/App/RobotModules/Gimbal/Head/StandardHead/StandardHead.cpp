#include "StandardHead.hpp"

#include "TopicRouter.hpp"

#include "GM6020.hpp"

#include "PidBasic.hpp"

#include "sdkconfig.h"
#include "Bsp_can.hpp"

extern canHandle HCAN1;

using namespace GIMBAL;
using namespace PINYMOTOR;

StandardHead::StandardHead()
{
    InitConfig_s pitchGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = 500.0f,
        .posPID = std::unique_ptr<PID>(
                new positonalPid(50.f, 0.f, 0.f, 1.f / 500.f, 0.f, 35.f, 0.f)),
        .velPID = std::unique_ptr<PID>(new positonalPid(
                0.03f, 0.05f, 0.f, 1.f / 500.f, 0.3f, 1.2f, 0.f)),
        .torqPID = nullptr,
        .isReverse = false,
    };
    motor_[0] = new DJIMOTOR::GM6020("Pitch", std::move(pitchGM6020Config));

    InitConfig_s yawGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 2,
        .txFreq = 500.0f,
        .posPID = std::unique_ptr<PID>(
                new positonalPid(50.0f, 0.f, 0.f, 1.f / 500.f, 0.f, 35.f, 0.f)),
        .velPID = std::unique_ptr<PID>(new positonalPid(
                0.05f, 0.05f, 0.000001f, 1.f / 500.f, 0.3f, 1.2f, 0.f)),
        .torqPID = nullptr,
        .isReverse = false,
    };
    motor_[1] = new DJIMOTOR::GM6020("Yaw", std::move(yawGM6020Config));

    this->insSub_ = new Subscriber<INS_SYS::INSData_s>(
            4, &TopicRouter::instance().insTopic, &insData_);
}

void StandardHead::stop()
{
    for (auto &i : motor_) {
        i->cmd(MotorCmdType_e::OFF);
    }
}

void StandardHead::enter()
{
    for (auto &i : motor_) {
        i->cmd(MotorCmdType_e::ON);
    }
}

void StandardHead::update()
{
    this->insSub_->receive();
    // TODO: vision update
}

PINYMOTOR::IMotor *StandardHead::joint(uint8_t _jId) const
{
    if (_jId > MOTOR_NUM - 1) {
        LOG::error("StandardHead", "Invalid joint ID: %d", _jId);
        return nullptr;
    } else {
        return motor_[_jId];
    }
}
