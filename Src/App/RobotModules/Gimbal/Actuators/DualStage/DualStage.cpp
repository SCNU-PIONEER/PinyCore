#include "sdkconfig.h"
#include "Bsp_uart.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "MiniPCPacketImpl.hpp"
#include "MotorCommonMacros.hpp"
#include "DualStage.hpp"
#include "PidBasic.hpp"
#include "GM6020.hpp"
#include "DM4310.hpp"

#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"
#include "./State/PatrolState.hpp"
#include "./State/AutoState.hpp"
#include "./State/NavState.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>


extern canHandle HCAN1;

using namespace GIMBAL;
using namespace PINYMOTOR;

static FILTER::LPF gimbalFilterOutput[3]{
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 5.f),
    FILTER::LPF(1000.f, 4.f),
};

DualStage::DualStage()
{
    for (uint8_t i = 0; i < 3; i++) {
        gimbalFilterOutput_._[i] = &gimbalFilterOutput[i];
    }

    isGimbalResverse.Gim_s.Pitch = DualStage::PITCH_REVERSE;
    isGimbalResverse.Gim_s.FirstYaw = DualStage::FIRST_YAW_REVERSE;
    isGimbalResverse.Gim_s.SecondYaw = DualStage::SECOND_YAW_REVERSE;

    InitConfig_s pitchDM4310Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 1,
        .txFreq = PITCH_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = PITCH_REVERSE,
    };
    motors_.Gim_s.Pitch = new DMMOTOR::DM4310("Pitch", pitchDM4310Config);
    motors_.Gim_s.Pitch->setZeroAng(PITCH_LAUNCH_ANG);
    LOG::info("Gimbal", "Pitch zeros has set to %f", motors_.Gim_s.Pitch->data().zeroAng);

    InitConfig_s firstYawDM4310Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN3),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 2,
        .txFreq = FIRST_YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = FIRST_YAW_REVERSE,
    };
    motors_.Gim_s.FirstYaw = new DMMOTOR::DM4310("BigYaw", firstYawDM4310Config);
    // motors_.Gim_s.FirstYaw->setZeroAng();
    motors_.Gim_s.FirstYaw->setZeroAng(FIRST_YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Big yaw zeros has set to %f", motors_.Gim_s.FirstYaw->data().zeroAng);

    InitConfig_s secondYawGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 3,
        .txFreq = SECOND_YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = SECOND_YAW_REVERSE,
    };
    motors_.Gim_s.SecondYaw = new DJIMOTOR::GM6020("SmallYaw", secondYawGM6020Config);
    motors_.Gim_s.SecondYaw->setZeroAng(SECOND_YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Small yaw zeros has set to %f", motors_.Gim_s.SecondYaw->data().zeroAng);

    this->stateFactory_.addState(FSMState_e::RUN, std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::LAUNCH, std::make_unique<LaunchState>(this));
    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    this->stateFactory_.addState(FSMState_e::PATROL, std::make_unique<PatrolState>(this));
    this->stateFactory_.addState(FSMState_e::AUTO, std::make_unique<AutoState>(this));
    this->stateFactory_.addState(FSMState_e::NAV, std::make_unique<NavState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));
}

void DualStage::stop()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::OFF);
    }
}

void DualStage::enter()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::ON);
    }
}

void DualStage::update(void *_param)
{
    this->insSub_->receive();

    if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) == pdTRUE) {
    };

    v2eData = miniPC_.data().v2eData;
    n2eData = miniPC_.data().n2eData;
    d2eData = miniPC_.data().d2eData;

    for (uint8_t i = 0; i < 3; i++) {
        curMotorAng_._[i] = motors_._[i]->posNorm(); // 0~2*pi
    }
    //ins -pi~pi
    curImuAng_.Gim_s.Pitch = insData_.pitch;
    curImuAng_.Gim_s.SecondYaw = insData_.yaw; //云台imu在小yaw上
    curImuAng_.Gim_s.FirstYaw = getMinorArc(curImuAng_.Gim_s.SecondYaw, rangeConversion(curMotorAng_.Gim_s.SecondYaw));
    compensateImuAngle();

    loadChassisMsg(_param);
    loadShooterMsg(_param);

    sendVisionData();
    sendNavData();
    sendDecesionData();

    //与云台控制无关，debug使用
    debug.visionControl = v2eData.control;
    debug.visionFire = v2eData.fire;
    debug.visionYaw = v2eData.yaw;
    debug.visionPitch = v2eData.pitch;
    debug.visionAngOver = isVisionAngOver_;

    this->stateFactory_.update();
}

void DualStage::loadChassisMsg(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg, 0) == pdTRUE) {
    };

    // cmsg.state = CHASSIS::FSMState_e::STOP;
    chassisCtrlTxPacket_.setData().state = static_cast<uint8_t>(cmsg.state);
    chassisCtrlTxPacket_.setData().gimbalReset = isLaunched_;

    if (msg.state == FSMState_e::NAV || msg.state == FSMState_e::AUTO) {
        chassisCtrlTxPacket_.setData().vx = cmsg.vx + n2eData.vx;
        chassisCtrlTxPacket_.setData().vy = cmsg.vy + n2eData.vy;
        chassisCtrlTxPacket_.setData().wz = cmsg.wz + n2eData.wz;
        if (msg.state == FSMState_e::NAV) {
            msg.wYaw = 0.5f;
        }
    } else if (msg.state == FSMState_e::RUN) {
        chassisCtrlTxPacket_.setData().vx = cmsg.vx;
        chassisCtrlTxPacket_.setData().vy = cmsg.vy;
        chassisCtrlTxPacket_.setData().wz = cmsg.wz;
        n2eData = {};
    }

    chassisCtrlTxPacket_.setData().deltaYaw = motors_.Gim_s.FirstYaw->posNorm();
    chassisCtrlTxPacket_.setData().sentryPosture = static_cast<uint8_t>(miniPC_.data().d2eData.posture);
}

void DualStage::loadShooterMsg(void *_param)
{
    ArmorLoaderMsg_s almsg{};
    if (xQueueReceive((((MsgBus_s *)_param)->armorLoaderQueue), &almsg, 0) == pdTRUE) {
        shooterCtrlTxPacket_.setData().state = static_cast<uint8_t>(almsg.state);
        shooterCtrlTxPacket_.setData().fire = almsg.fire;
    }
    if (msg.state == FSMState_e::RUN) {
        shooterCtrlTxPacket_.setData().fire &= (v2eData.control) ? v2eData.fire : true;
    } else if (msg.state == FSMState_e::NAV || msg.state == FSMState_e::AUTO) {
        shooterCtrlTxPacket_.setData().fire &= v2eData.fire;
    }
}

void DualStage::sendVisionData()
{ // vision send
    e2vData.roll = insData_.roll;
    e2vData.pitch = insData_.pitch;
    e2vData.yaw = getMinorArc(insData_.yaw, -this->deltaSYawArc_);
    // e2vData.yaw = insData_.yaw;

    //装裁判系统数据
    e2vData.detectColor = isColor(refereeRxPacket_.data().robotId);
    e2vData.isPlay = refereeRxPacket_.data().isPlay;
    e2vData.bulletSpeed = bulletSpdRxPacket_.data().shootSpeed;
}

void DualStage::sendNavData()
{ //nav send
    e2nData.yaw = motors_.Gim_s.FirstYaw->posNorm();
}

void DualStage::sendDecesionData()
{ //decesion send
    //装裁判系统数据
    e2dData.myHP = refereeRxPacket_.data().HP;
    e2dData.gameTime = refereeRxPacket_.data().gameTime;
    e2dData.gameProgress = refereeRxPacket_.data().gameProgress;

    e2dData.currentChassisEnergy = -1;
    e2dData.outpostStatus = -1;
    e2dData.enemyBaseStatus = -1;
    e2dData.enemyOutpostStatus = -1;
    e2dData.eventData = -1;
}

void DualStage::compensateImuAngle()
{
    float tempComAng;
    if (this->isConfirmYawZero_) {
        //控制大yaw
        this->curImuAng_.Gim_s.SecondYaw = getMinorArc(this->curImuAng_.Gim_s.SecondYaw, -this->deltaSYawArc_);
        this->curImuAng_.Gim_s.FirstYaw = getMinorArc(this->curImuAng_.Gim_s.FirstYaw, -this->deltaBYawArc_);
        //与gimbal类保持相同
        this->ctrlYawAng_ = this->curImuAng_.Gim_s.FirstYaw;
    } else {
        // confirm zero
        if (motors_.Gim_s.SecondYaw->posNorm() != 0.f && motors_.Gim_s.FirstYaw->posNorm() != 0.f) {
            // TODO: prefer to globalState
            /*实际在获取上电后imu确定的坐标系与车体坐标系正方向的差角,当前imu在小yaw上.
            小yaw+大yaw=imu坐标系与车体坐标系正方向的差角*/
            tempComAng = motors_.Gim_s.SecondYaw->posNorm() + motors_.Gim_s.FirstYaw->posNorm();
            //0~2pi
            tempComAng = PINYMOTOR::rangeMap(tempComAng);
            this->deltaSYawArc_ = SECOND_YAW_REVERSE ?
                                          getMinorArc(0.f, rangeConversion(motors_.Gim_s.SecondYaw->posNorm())) :
                                          getMinorArc(rangeConversion(motors_.Gim_s.SecondYaw->posNorm()), 0.f);
            this->deltaBYawArc_ = FIRST_YAW_REVERSE ? getMinorArc(0.f, tempComAng) : getMinorArc(tempComAng, 0.f);
            this->isConfirmYawZero_ = true;
            // this->isSetBYZeroAng_ = true;
        } else {
            this->isConfirmYawZero_ = false;
        }
    }
}

void DualStage::updateBaseYaw() {}

float outMaxRunVel[3];
float outMaxRunTorq[3];
void DualStage::ctrl(DualStage_u<float> _pos, DualStage_u<PositionalPid *> _angPID,
                     DualStage_u<PositionalPid *> _velPID)
{
    float del[3], vel[3], torq[3];
    for (uint8_t i = 0; i < 3; i++) {
        del[i] = getMinorArc(_pos._[i], motors_._[i]->posNorm());
        vel[i] = _angPID._[i]->calc(del[i], 0);
        outMaxRunVel[i] = vel[i];
        torq[i] = _velPID._[i]->calc(vel[i], motors_._[i]->vel());
        outMaxRunTorq[i] = torq[i];
        motors_._[i]->cmdTorq(gimbalFilterOutput_._[i]->process(torq[i]));
    }
    // motors_.Gim_s.FirstYaw->cmdTorq(0.f);
    // motors_.Gim_s.SecondYaw->cmdTorq(0.f);
    // motors_.Gim_s.Pitch->cmdTorq(0.f);
}

void DualStage::ctrlSmooth(DualStage_u<float> _pos, DualStage_u<float> _step, DualStage_u<PositionalPid *> _angPID,
                           DualStage_u<PositionalPid *> _velPID)
{
    float del[3], vel[3], torq[3];
    for (uint8_t i = 0; i < 3; i++) {
        del[i] = getMinorArc(_pos._[i], motors_._[i]->posNorm());
        vel[i] = _angPID._[i]->calc(fmax(-_step._[i], fmin(del[i], _step._[i])), 0);
        torq[i] = _velPID._[i]->calc(vel[i], motors_._[i]->vel());
        motors_._[i]->cmdTorq(gimbalFilterOutput_._[i]->process(torq[i]));
    }
    // motors_.Gim_s.FirstYaw->cmdTorq(0.f);
    // motors_.Gim_s.SecondYaw->cmdTorq(0.f);
    // motors_.Gim_s.Pitch->cmdTorq(0.f);
}

//前馈pid,暂时无用
volatile float accDebug = 0;
volatile float torqDebug = 0;
volatile float velDebug = 0;
void DualStage::ctrlSecondYawAuto(PID *_angPID, PID *_velPID, PID *_velFeedbackPID, float _pos)
{
    float insYawDelArc = getMinorArc(_pos, ins().yaw);
    float pos = rangeMap(motors_.Gim_s.SecondYaw->posNorm() + insYawDelArc);
    float del = getMinorArc(pos, motors_.Gim_s.SecondYaw->posNorm());
    float vel = _angPID->calc(del, 0);
    velDebug = vel;
    float acc = _velPID->calc(vel, insData_.body.gz) + _velFeedbackPID->calc(v2eData.yawVel, insData_.body.gz);
    accDebug = acc += v2eData.yawAcc; // forward feedbac
    float torq = calcYawTorq(insData_.body.gz, acc);
    torqDebug = torq = std::clamp(torq, -DJIMOTOR::GM6020::TORQ_MAX, DJIMOTOR::GM6020::TORQ_MAX);
    motors_.Gim_s.SecondYaw->cmdTorq(gimbalFilterOutput_.Gim_s.SecondYaw->process(torq));
}

float DualStage::calcYawTorq(float _vel, float _acc)
{
    static constexpr auto SIGN = [](float _v) {
        if (_v > 0.f)
            return 1.f;
        else if (_v < 0.f)
            return -1.f;
        else
            return 0.f;
    };

    float torq = (YAW_INERTIA * _acc) + (YAW_FRICTION * _vel) + (YAW_COULOMB * SIGN(_vel));
    return torq;
}
