#include "sdkconfig.h"
#include "Gimbal.hpp"
#include "MiniPCPacketImpl.hpp"
#include "MotorCommonMacros.hpp"
#include "DualStage.hpp"
#include "PidBasic.hpp"
#include "DM4310.hpp"
#include "ArmorLoader.hpp"
#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"
#include "./State/AutoState.hpp"
#include "./State/NavState.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

extern canHandle HCAN1;

using namespace GIMBAL;
using namespace PINYMOTOR;

static FILTER::LPF gimbalFilterOutput[3]{
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 10.f),
    FILTER::LPF(1000.f, 4.f),
};

DualStage::DualStage()
{
    for (uint8_t i = 0; i < 3; i++) {
        gimbalFilterOutput_._[i] = &gimbalFilterOutput[i];
    }

    isGimbalResverse.Pitch = DualStage::PITCH_REVERSE;
    isGimbalResverse.FirstYaw = DualStage::FIRST_YAW_REVERSE;
    isGimbalResverse.SecondYaw = DualStage::SECOND_YAW_REVERSE;

    InitConfig_s pitchDM4310Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .motorType = SupportMotor_e::DA_MIAO,
        .offsetId = 1,
        .txFreq = PITCH_MOTOR_FREQ,
        .isReverse = PITCH_REVERSE,
    };
    motors_.Pitch = new DMMOTOR::DM4310("Pitch", pitchDM4310Config, DMMOTOR::WorkMode_e::MIT_TT);
    motors_.Pitch->setZeroAng(PITCH_LAUNCH_ANG);
    LOG::info("Gimbal", "Pitch zeros has set to %f", motors_.Pitch->data().zeroAng);

    InitConfig_s firstYawDM4310Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN3),
        .comType = ComType_e::CAN,
        .motorType = SupportMotor_e::DA_MIAO,
        .offsetId = 2,
        .txFreq = FIRST_YAW_MOTOR_FREQ,
        .isReverse = FIRST_YAW_REVERSE,
    };
    motors_.FirstYaw = new DMMOTOR::DM4310("BigYaw", firstYawDM4310Config, DMMOTOR::WorkMode_e::MIT_TT);

    bigYawZero_ = motors_.FirstYaw->data().rawAng; //记录大yaw初始零点位置,用于缓启动阶段
    // motors_.FirstYaw->setZeroAng(bigYawZero_);
    motors_.FirstYaw->setZeroAng(FIRST_YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Big yaw zeros has set to %f", motors_.FirstYaw->data().zeroAng);

    InitConfig_s secondYawDM4310Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .motorType = SupportMotor_e::DA_MIAO,
        .offsetId = 3,
        .txFreq = SECOND_YAW_MOTOR_FREQ,
        .isReverse = SECOND_YAW_REVERSE,
    };
    motors_.SecondYaw = new DMMOTOR::DM4310("SmallYaw", secondYawDM4310Config, DMMOTOR::WorkMode_e::MIT_TT);

    motors_.SecondYaw->setZeroAng(SECOND_YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Small yaw zeros has set to %f", motors_.SecondYaw->data().zeroAng);

    this->stateFactory_.addState(FSMState_e::RUN, std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::LAUNCH, std::make_unique<LaunchState>(this));
    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    this->stateFactory_.addState(FSMState_e::AUTO, std::make_unique<AutoState>(this));
    this->stateFactory_.addState(FSMState_e::NAV, std::make_unique<NavState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));

#if EXTENSION_SUPERCAP
    bigYawCap_.limitCharge();
#endif
}

void DualStage::stop()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::OFF);
    }
    for (auto &i : gimbalFilterOutput_._) {
        i->reset();
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

    if (miniPC_.pcState == MiniPC::PCState_e::ONLINE) {
        v2eData = miniPC_.data().v2eData;
        n2eData = miniPC_.data().n2eData;
        d2eData = miniPC_.data().d2eData;
    } else {
        v2eData = {};
        n2eData = {};
        d2eData = {};
    }

    for (uint8_t i = 0; i < 3; i++) {
        curMotorAng_._[i] = motors_._[i]->posNorm(); // 0~2*pi
    }
    //ins -pi~pi
    curImuAng_.Pitch = insData_.pitch;
    curImuAng_.SecondYaw = insData_.yaw; //云台imu在小yaw上
    curImuAng_.FirstYaw = getMinorArc(curImuAng_.SecondYaw, rangeConversion(curMotorAng_.SecondYaw));

    loadChassisMsg(_param);
    loadShooterMsg(_param);

    sendVisionData();
    sendNavData();
    sendDecesionData();
    loadSentryCmd();

    msg.wYaw = std::clamp(msg.wYaw, -DualStage::MAX_BYAW_SPEED, DualStage::MAX_BYAW_SPEED);
    if (d2eData.isBumpArea) {
        v2eData.control = false; //减速带关闭自瞄
    }
    //与云台控制无关，debug使用
    debug.visionControl = v2eData.control;
    debug.visionFire = v2eData.fire;
    debug.visionYaw = v2eData.yaw;
    debug.visionPitch = v2eData.pitch;

#if EXTENSION_SUPERCAP
    bigYawCap_.enable();
    bigYawCap_.setChargePowerLimit(50);
    bigYawCap_.setChargeEnergySlack(60);
    bigYawCap_.setChargelimitRatio(0.75f);
    bigYawCap_.task();
#endif

    this->stateFactory_.update();
}

void DualStage::loadChassisMsg(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg, 0) == pdTRUE) {
    };

    //cmsg.state = CHASSIS::FSMState_e::STOP;
    chassisCtrlTxPacket_.setData().state = static_cast<uint8_t>(cmsg.state);
    chassisCtrlTxPacket_.setData().gimbalReset = isLaunched_;

    if (msg.state == FSMState_e::NAV || msg.state == FSMState_e::AUTO) {
        chassisCtrlTxPacket_.setData().vx = n2eData.vx; //cmsg.vx + n2eData.vx;
        chassisCtrlTxPacket_.setData().vy = n2eData.vy; //cmsg.vy + n2eData.vy;
        chassisCtrlTxPacket_.setData().wz = n2eData.wz; //cmsg.wz + n2eData.wz;
        if (msg.state == FSMState_e::NAV) {
            msg.wYaw = n2eData.wYaw; //导航模式下，云台控制权交给导航
        } else {
            msg.wYaw = 0.0f;
        }
    } else if (msg.state == FSMState_e::RUN) {
        chassisCtrlTxPacket_.setData().vx = cmsg.vx;
        chassisCtrlTxPacket_.setData().vy = cmsg.vy;
        chassisCtrlTxPacket_.setData().wz = cmsg.wz;
        // n2eData = {};
    }

    chassisCtrlTxPacket_.setData().deltaYaw = curMotorAng_.FirstYaw;
    chassisCtrlTxPacket_.setData().useSuperCap = d2eData.useSuperCap;
}

void DualStage::loadShooterMsg(void *_param)
{
    ArmorLoaderMsg_s almsg{};
    if (xQueueReceive((((MsgBus_s *)_param)->armorLoaderQueue), &almsg, 0) == pdTRUE) {
        if (d2eData.bufferMode != 0 && almsg.state != ARMORLOADER::FSMState_e::STOP) {
            almsg.state = ARMORLOADER::FSMState_e::SINGLE;
        }
        shooterCtrlTxPacket_.setData().state = static_cast<uint8_t>(almsg.state);
        enFire = almsg.fire;
    }

    if (msg.state == FSMState_e::RUN) {
        if (v2eData.control) {
            enFire &= (v2eData.fire);
        } else {
            enFire &= true;
        }
    } else if (msg.state == FSMState_e::NAV) {
        enFire &= v2eData.fire;
    }

    shooterCtrlTxPacket_.setData().fire = enFire;
}

void DualStage::loadSentryCmd()
{
    sentryCmdTxPacket_.setData().midSentryCmd = d2eData.upSentryCmd;
    sentryCmdTxPacket_.setData().isBumpArea = d2eData.isBumpArea;
}

void DualStage::sendVisionData()
{ // vision send
    e2vData.roll = insData_.roll;
    e2vData.pitch = insData_.pitch;
    e2vData.yaw = insData_.yaw;

    //装裁判系统数据
    e2vData.detectColor = isColor(refereeRxPacket_.data().robotId);
    // e2vData.isPlay = refereeRxPacket_.data().isPlay;
    e2vData.isPlay = false; //关闭视觉录包,留出内存给导航
    e2vData.bulletSpeed = bulletSpdRxPacket_.data().shootSpeed;

    e2vData.taskMode = d2eData.bufferMode;
}

void DualStage::sendNavData()
{                                        //nav send
    e2nData.yaw = curMotorAng_.FirstYaw; //雷达在大yaw上
}

void DualStage::sendDecesionData()
{ //decesion send
    //装裁判系统数据
    e2dData.myHP = refereeRxPacket_.data().HP;
    e2dData.gameTime = refereeRxPacket_.data().gameTime;
    e2dData.gameProgress = refereeRxPacket_.data().gameProgress;

    e2dData.upSentryInfo = sentryInfoRxPacket_.data().midSentryInfo.info2;
    e2dData.currentChassisEnergy = decesionRxPacket1_.data().currentChassisEnergy;
    e2dData.allyOutpostHP = decesionRxPacket1_.data().allyOutpostHP;
    e2dData.allyBaseHP = decesionRxPacket1_.data().allyBaseHP;
    e2dData.hurtHpDeductionReason = sentryInfoRxPacket_.data().hurtHpDeductionReason;
    e2dData.bulletRemainingNum17mm = decesionRxPacket1_.data().bulletRemainingNum17mm;

    // e2dData.targetPositionX = decesionRxPacket2_.data().mapCommand.targetPositionX;
    // e2dData.targetPositionY = decesionRxPacket2_.data().mapCommand.targetPositionY;
}

/**
 * @brief 缓启动专用
 * 
 * @param _pos 电机系角度
 * @param _step 缓启动步长
 * @param _angPID 
 * @param _velPID 
 */
void DualStage::ctrlSmooth(DualStage_u<float> _pos, DualStage_u<float> _step, DualStage_u<PositionalPid *> _angPID,
                           DualStage_u<PositionalPid *> _velPID)
{
    float del[3], vel[3], torq[3];
    for (uint8_t i = 0; i < 3; i++) {
        del[i] = getMinorArc(_pos._[i], curMotorAng_._[i]);
        vel[i] = _angPID._[i]->calc(fmax(-_step._[i], fmin(del[i], _step._[i])), 0);
        torq[i] = _velPID._[i]->calc(vel[i], motors_._[i]->vel());
        motors_._[i]->cmdTorq(gimbalFilterOutput_._[i]->process(torq[i]));
    }
    // motors_.FirstYaw->cmdTorq(0.f);
    // motors_.SecondYaw->cmdTorq(0.f);
    // motors_.Pitch->cmdTorq(0.f);
}

/**
 * @brief 输入imu角度,传入电机控制函数
 * 
 * @param _insPos imu系角度
 * @return DualStage_u<float> 电机系期望角度
 */
DualStage_u<float> DualStage::ctrl(DualStage_u<float> _insPos)
{
    DualStage_u<float> delArc = {}, cmdMotorAng = {};
    for (uint8_t i = 0; i < 3; i++) {
        delArc._[i] = getMinorArc(_insPos._[i], curImuAng_._[i]);
        cmdMotorAng._[i] = rangeMap(curMotorAng_._[i] + delArc._[i]);
    }
    return cmdMotorAng;
}

float outMaxRunVel[3];
float outMaxRunTorq[3];
/**
 * @brief 云台的电机控制接口，输入电机系角度，输出电机转矩
 * 
 * @param _expMotorAng 电机系角度
 * @param _angPID 
 * @param _velPID 
 */
void DualStage::ctrlMotor(DualStage_u<float> _expMotorAng, DualStage_u<PositionalPid *> _angPID,
                          DualStage_u<PositionalPid *> _velPID)
{
    float del[3], vel[3], torq[3];
    for (uint8_t i = 0; i < 3; i++) {
        del[i] = getMinorArc(_expMotorAng._[i], curMotorAng_._[i]);
        vel[i] = _angPID._[i]->calc(del[i], 0);
        torq[i] = _velPID._[i]->calc(vel[i], motors_._[i]->vel());
        motors_._[i]->cmdTorq(gimbalFilterOutput_._[i]->process(torq[i]));
    }
    for (uint8_t i = 0; i < 3; i++) {
        outMaxRunVel[i] = vel[i];
        outMaxRunTorq[i] = torq[i];
    }
    // motors_.FirstYaw->cmdTorq(0.f);
    // motors_.SecondYaw->cmdTorq(0.f);
    // motors_.Pitch->cmdTorq(0.f);
}

//前馈pid,小yaw用
volatile float accDebug = 0;
volatile float torqDebug = 0;
volatile float velDebug = 0;
void DualStage::ctrlYawAuto(DualStage_u<float> _insPos, PID *_velFeedbackPID, DualStage_u<PositionalPid *> _angPID,
                            DualStage_u<PositionalPid *> _velPID)
{
    float delArc[2], cmdMotorAng[2], del[2], vel[2], torq[2]; //pitch,bigYaw
    for (uint8_t i = 0; i < 2; i++) {
        delArc[i] = getMinorArc(_insPos._[i], curImuAng_._[i]);
        cmdMotorAng[i] = rangeMap(curMotorAng_._[i] + delArc[i]);
        del[i] = getMinorArc(cmdMotorAng[i], curMotorAng_._[i]);
        vel[i] = _angPID._[i]->calc(del[i], 0);
        torq[i] = _velPID._[i]->calc(vel[i], motors_._[i]->vel());
        motors_._[i]->cmdTorq(gimbalFilterOutput_._[i]->process(torq[i]));
    }
    //sYaw
    float insYawDelArc = getMinorArc(_insPos.SecondYaw, ins().yaw);
    float sVel = _angPID.SecondYaw->calc(insYawDelArc, 0);
    float acc = _velPID.SecondYaw->calc(sVel, insData_.earth.gz) +
                _velFeedbackPID->calc(v2eData.yawVel, insData_.earth.gz) + v2eData.yawAcc;
    float sTorq = calcYawTorq(insData_.earth.gz, acc);
    sTorq = std::clamp(sTorq, -DMMOTOR::DM4310::TORQ_MAX / 2, DMMOTOR::DM4310::TORQ_MAX / 2);
    motors_.SecondYaw->cmdTorq(gimbalFilterOutput_.SecondYaw->process(sTorq));

    velDebug = sVel;
    accDebug = acc - v2eData.yawAcc; // forward feedbac
    torqDebug = sTorq;
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
