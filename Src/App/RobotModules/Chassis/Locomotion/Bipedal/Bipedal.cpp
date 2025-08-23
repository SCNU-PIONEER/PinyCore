#include "Bipedal.hpp"
#include "M3508.hpp"
#include "DM8009.hpp"
#include "TopicRouter.hpp"
#include "dsp/fast_math_functions.h"
#include "sdkconfig.h"
#include "Bsp_can.hpp"
#include "Dwt.h"

#include "OSnew.hpp"

#include "CircleFunc.h"

#include "./State/StopState.hpp"
#include "./State/RunState.hpp"
#include "./State/LaunchState.hpp"

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
    motors_.WheelR = new DJIMOTOR::M3508("WheelR", rM3508Config);

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
    motors_.HipRF = new DMMOTOR::DM8009("HipRF", rfDM8009Config);
    motors_.HipRF->setZeroAng(RF_HIP_ZERO_ANG);

    InitConfig_s rbDM8009Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 2,
        .txFreq = 500.f,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr,
        .isReverse = false,
    };
    motors_.HipRB = new DMMOTOR::DM8009("HipRB", rbDM8009Config);
    motors_.HipRB->setZeroAng(RB_HIP_ZERO_ANG);

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
    motors_.WheelL = new DJIMOTOR::M3508("WheelL", lM3508Config);

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
    motors_.HipLF = new DMMOTOR::DM8009("HipLF", lfDM8009Config);
    motors_.HipLF->setZeroAng(LF_HIP_ZERO_ANG);

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
    motors_.HipLB = new DMMOTOR::DM8009("HipLB", lbDM8009Config);
    motors_.HipLB->setZeroAng(LB_HIP_ZERO_ANG);

    legL_.init(motors_.HipLF, motors_.HipLB, motors_.WheelL);
    legR_.init(motors_.HipRF, motors_.HipRB, motors_.WheelR);

    stateFactory.addState(static_cast<uint8_t>(FSMState_e::RUN),
                          std::make_unique<RunState>(this));
    stateFactory.addState(static_cast<uint8_t>(FSMState_e::LAUNCH),
                          std::make_unique<LaunchState>(this));
    stateFactory.addState(static_cast<uint8_t>(FSMState_e::STOP),
                          std::make_unique<StopState>(this));
    stateFactory.init(
            stateFactory.getNextState(static_cast<uint8_t>(FSMState_e::STOP)));
}

void Bipedal::stop()
{
    legL_.stop();
    legR_.stop();

    rollAngPid_.reset();
    rollVelPid_.reset();

    hipAngPidL_.reset();
    hipVelPidL_.reset();
    hipAngPidR_.reset();
    hipVelPidR_.reset();

    wzVelPid_.reset();
    dsPid_.reset();
}

void Bipedal::enter()
{
    legL_.enter();
    legR_.enter();

    s_ = 0;
    setRoll_ = 0;
}

void Bipedal::update(void *_param)
{
    // update dt
    dt_ = dwt_get_dt(&dtCnt_);

    // update msg
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) ==
        pdTRUE) {
    };

    // update insData
    insSub_->receive();

    // update ax az
    float sinPitch = arm_sin_f32(insData_.pitch);
    float cosPitch = arm_cos_f32(insData_.pitch);
    float cosRoll = arm_cos_f32(insData_.roll);

    float g = WheelLegged::GRAVITY;
    float axFromX = (insData_.body.ax - (g * -sinPitch)) * cosPitch;
    float axFromZ =
            (insData_.body.az - (g * cosPitch * cosRoll)) * sinPitch * cosRoll;
    float axFromINS = axFromX + axFromZ;

    float azFromX = (insData_.body.ax - (g * -sinPitch)) * -sinPitch;
    float azFromZ =
            (insData_.body.az - (g * cosPitch * cosRoll)) * cosPitch * cosRoll;
    float azFromINS = azFromX + azFromZ;

    // update leg
    legL_.stateUpdate(dt_);
    legR_.stateUpdate(dt_);

    legL_.isFloating = detector_.isFloating(legL_.calcFn(az_));
    legR_.isFloating = detector_.isFloating(legR_.calcFn(az_));

    // update odom
    float vxFromWheel =
            (motors_.WheelL->vel() + motors_.WheelR->vel()) * W_RADIUS / 2.f;
    odom_.update(dt_, vxFromWheel, axFromINS, yaw_);
    ds_ = odom_.normVel();
    s_ = (detector_.isStop(this->msg.vx, ds_)) ? s_ + (ds_ * dt_) : 0;
    as_ = odom_.normAcc();

    // update other state
    float del = insData_.yaw + PI - yawZero_;       // map to [0,2PI]
    yaw_ = (del < 0 ? del + (2.f * PI) : del) - PI; // map back to [-PI,PI]

    wz_ = insData_.body.gz; // TODO: earth.gz
    wx_ = insData_.body.gx; // TODO: earth.gx
    az_ = azFromINS;

    // update FSM
    this->stateFactory.update();
}

void Bipedal::setYawZero(float _yawZero) { yawZero_ = _yawZero; }

void Bipedal::setRoll(float _roll)
{
    setRoll_ = _roll;
    float dRoll = rollAngPid_.calc(setRoll_, insData_.roll);
    float delta = rollVelPid_.calc(dRoll, wx_);
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

void Bipedal::ctrlBal(const WBRController::States_u &_ref)
{
    WBRController ::States_u cur = { .s = s_,
                                     .ds = ds_,
                                     .yaw = yaw_,
                                     .dyaw = wz_,
                                     .phil = legL_.state().phi0,
                                     .dphil = legL_.state().dphi0,
                                     .phir = legR_.state().phi0,
                                     .dphir = legR_.state().dphi0,
                                     .q = insData_.pitch,
                                     .dq = insData_.body.gy };
    balWBRCtrl_.load(_ref, cur);
    Matrix<4, 1> u = balWBRCtrl_.solve(legL_.state().L0, legR_.state().L0,
                                       legL_.isFloating, legR_.isFloating);
    legL_.setTw(u[0][0]);
    legL_.setTp(u[1][0]);
    legR_.setTw(u[2][0]);
    legR_.setTp(u[3][0]);

    legL_.ctrl();
    legR_.ctrl();
}

void Bipedal::ctrlBal(const WLController::States_u &_ref, float _wz)
{
    ctrlBal(_ref, _ref, _wz);
}

void Bipedal::ctrlBal(const WLController::States_u &_refL,
                      const WLController::States_u &_refR, float _wz)
{
    WLController::States_u curL = {
        .phi = legL_.state().phi0,
        .dphi = legL_.state().dphi0,
        .s = s_,
        .ds = ds_,
        .q = insData_.pitch,
        .dq = insData_.body.gy,
    };

    WLController::States_u curR = {
        .phi = legR_.state().phi0,
        .dphi = legR_.state().dphi0,
        .s = s_,
        .ds = ds_,
        .q = insData_.pitch,
        .dq = insData_.body.gy,
    };
    balWLCtrlL_.load(_refL, curL);
    balWLCtrlR_.load(_refR, curR);
    Matrix<2, 1> uL = balWLCtrlL_.solve(legL_.state().L0, legL_.isFloating);
    Matrix<2, 1> uR = balWLCtrlR_.solve(legR_.state().L0, legR_.isFloating);
    float tw2 = wzVelPid_.calc(_wz, wz_);
    legL_.setTw(uL[0][0] + tw2);
    legL_.setTp(uL[1][0]);
    legR_.setTw(uR[0][0] - tw2);
    legR_.setTp(uR[1][0]);

    legL_.ctrl();
    legR_.ctrl();
}

void Bipedal::ctrlBal(const STController::States_u &_ref, float _wz)
{
    STController::States_u cur = {
        .s = s_,
        .ds = ds_,
        .q = insData_.pitch,
        .dq = insData_.body.gy,
    };
    balSTCtrl_.load(_ref, cur);
    float lAvg = (legL_.state().L0 + legR_.state().L0) / 2.f;
    bool isf = legL_.isFloating & legR_.isFloating;
    Matrix<1, 1> uDiv = balSTCtrl_.solve(lAvg, isf) / 2.f;
    float tw2 = wzVelPid_.calc(_wz, wz_);
    legL_.setTw(uDiv[0][0] + tw2);
    legR_.setTw(uDiv[0][0] - tw2);

    legL_.ctrl();
    legR_.ctrl();
}

void Bipedal::ctrlNoBal(float _vx, float _wz)
{
    float tw1 = dsPid_.calc(_vx, ds_);
    float tw2 = wzVelPid_.calc(_wz, wz_);
    legL_.setTw(tw1 + tw2);
    legR_.setTw(tw1 - tw2);

    legL_.ctrl();
    legR_.ctrl();
}

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

void Bipedal::ctrlHipRot(float _dphi) { ctrlHipRot(_dphi, _dphi); }
void Bipedal::ctrlHipRot(float _dphiL, float _dphiR)
{
    ctrlHip(legL_.state().phi0 + _dphiL, legR_.state().phi0 + _dphiR);
}

bool Bipedal::isTumble()
{
    return detector_.detectTumble(legL_.state(), legR_.state(), insData_);
}

bool Bipedal::isFreeze() { return detector_.detectFreeze(insData_); }

bool Bipedal::isOverturn() { return insData_.body.az > 0.1f; }

bool Bipedal::isLaunched(float _setPhi, float _phiTh, float _setL, float _lTh)
{
    const auto &leftState = legL_.state();
    const auto &rightState = legR_.state();

    return fabs(minorArc(_setPhi, leftState.phi0)) < _phiTh &&
           fabs(minorArc(_setPhi, rightState.phi0)) < _phiTh &&
           fabs(leftState.L0 - _setL) < _lTh &&
           fabs(rightState.L0 - _setL) < _lTh;
}

float Bipedal::yaw() const { return yaw_; }