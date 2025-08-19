#include "Bipedal.hpp"
#include "M3508.hpp"
#include "DM8009.hpp"
#include "TopicRouter.hpp"
#include "dsp/fast_math_functions.h"
#include "sdkconfig.h"
#include "Bsp_can.hpp"

#include "OSnew.hpp"

extern canHandle HCAN1; // wheel
extern canHandle HCAN2; // hip

using namespace CHASSIS::BIPEDAL;
using namespace PINYMOTOR;

Bipedal::Bipedal()
        : insSub_(new Subscriber<INS_SYS::INSData_s>(
                  4, &TopicRouter::instance().insTopic, &insData_))
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
    motor_[1]->setZeroAng(RF_HIP_ZERO_ANG);

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
    motor_[2]->setZeroAng(RB_HIP_ZERO_ANG);

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
    motor_[4]->setZeroAng(LF_HIP_ZERO_ANG);

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
    motor_[5]->setZeroAng(LB_HIP_ZERO_ANG);
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
    insSub_->receive();

    // temp
    float sinPitch = arm_sin_f32(insData_.pitch);
    float cosPitch = arm_cos_f32(insData_.pitch);
    float cosRoll = arm_cos_f32(insData_.roll);

    float g = WheelLegged::GRAVITY;
    float axFromX = (insData_.body.ax - (g * -sinPitch)) * cosPitch;
    float axFromZ =
            (insData_.body.az - (g * cosPitch * cosRoll)) * sinPitch * cosRoll;
    ax_ = axFromX + axFromZ;

    float azFromX = (insData_.body.ax - (g * -sinPitch)) * -sinPitch;
    float azFromZ =
            (insData_.body.az - (g * cosPitch * cosRoll)) * cosPitch * cosRoll;
    az_ = azFromX + azFromZ;

    legL_.stateUpdate(0.001f);
    legR_.stateUpdate(0.001f);

    legL_.detectFloating(az_);
    legR_.detectFloating(az_);
}

void Bipedal::setRoll(float _roll)
{
    setRoll_ = _roll;
    float dRoll = rollAngPid_.calc(setRoll_, insData_.roll);
    float delta = rollVelPid_.calc(dRoll, insData_.body.gx);
    setL(legL_.state().L0 + delta, legR_.state().L0 - delta);
}

void Bipedal::setL(float _l) { setL(_l, _l); }

void Bipedal::setL(float _lL, float _lR)
{
    legL_.setL(_lL);
    legR_.setL(_lR);
}

void Bipedal::setFd(float _fd) { setFd(_fd, _fd); }

void Bipedal::setFd(float _fdL, float _fdR)
{
    legL_.setFd(_fdL);
    legR_.setFd(_fdR);
}

void Bipedal::ctrlBal(const WLController::States_u &_ref, float _wz) {}

void Bipedal::ctrlBal(const WLController::States_u &_refL,
                      const WLController::States_u &_refR, float _wz)
{
}

void Bipedal::ctrlBal(const WBRController::States_u &_ref) {}

void Bipedal::ctrlBal(const STController::States_u &_ref) {}

void Bipedal::ctrlNoBal(float _vx, float _wz) {}

void Bipedal::ctrlHip(float _phi) { ctrlHip(_phi, _phi); }

void Bipedal::ctrlHip(float _phiL, float _phiR)
{
    float hipVelL = hipAngPidL_.calc(_phiL, legL_.state().phi0);
    float tpL = hipVelPidL_.calc(hipVelL, legL_.state().dphi0);
    legL_.setTp(tpL);

    float hipVelR = hipAngPidR_.calc(_phiR, legR_.state().phi0);
    float tpR = hipVelPidR_.calc(hipVelR, legR_.state().dphi0);
    legR_.setTp(tpR);

    legL_.ctrl();
    legR_.ctrl();
}
