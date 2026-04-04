#include "Swerve.hpp"
#include "Chassis.hpp"
#include "Referee.hpp"
#include "RefereeProt.hpp"
#include "Soc.hpp"
#include "dsp/fast_math_functions.h"
#include "sdkconfig.h"
#include "Bsp_can.hpp"
#include "PidBasic.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include "GM6020.hpp"
#include "M3508.hpp"


#include "QuadricycleController.hpp"
#include "./State/TestState.hpp"
#include "./State/StopState.hpp"
#include "./State/RunState.hpp"
#include "./State/SpinState.hpp"
#include "./State/TANKState.hpp"


extern canHandle HCAN1;
extern canHandle HCAN2;
extern canHandle HCAN3;

using namespace CHASSIS;
using namespace PINYMOTOR;

constexpr float F_PI = std::numbers::pi_v<float>;

//以下数组可以塞到类内。但不好用free master
//RF 0 ,LF 1 ,LB 2 ,RB 3

FILTER::LPFIIR3 swerveFilterOutput[4] = {
    { 500.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
    { 500.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
    { 500.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
    { 500.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
};

FILTER::LPF swerveFilterInput[4]{
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 4.f),
};

PositionalPid swervePosPid[4] = {
    PositionalPid(30.f, 0.f, 1.f, 1.f / 1000.f, 60.f, 120.f, 0.f),
    PositionalPid(30.f, 0.f, 1.f, 1.f / 1000.f, 60.f, 120.f, 0.f),
    PositionalPid(30.f, 10.f, 1.f, 1.f / 1000.f, 60.f, 120.f, 0.f),
    PositionalPid(30.f, 0.f, 1.f, 1.f / 1000.f, 60.f, 120.f, 0.f),
};
PositionalPid swerveVelPid[4] = {
    PositionalPid(0.1f, 0.f, 0.f, 1.f / 1000.f, 0.6f, 1.f, 0.f),
    PositionalPid(0.06f, 0.f, 0.f, 1.f / 1000.f, 0.6f, 1.f, 0.f),
    PositionalPid(0.06f, 0.f, 0.f, 1.f / 1000.f, 0.6f, 1.f, 0.f),
    PositionalPid(0.1f, 0.f, 0.f, 1.f / 1000.f, 0.6f, 1.f, 0.f),
};

PositionalPid wheelVelPid[4] = {
    PositionalPid(0.45f, 0.000f, 0.001f, 1.f / 1000.f, 1.f, 4.f, 0.1f),
    PositionalPid(0.45f, 0.000f, 0.001f, 1.f / 1000.f, 1.f, 4.f, 0.1f),
    PositionalPid(0.45f, 0.000f, 0.001f, 1.f / 1000.f, 1.f, 4.f, 0.1f),
    PositionalPid(0.45f, 0.000f, 0.001f, 1.f / 1000.f, 1.f, 4.f, 0.1f),
};

Swerve::Swerve()
{
    insSub_ = new Subscriber<INS_SYS::INSData_s>(4, &TopicRouter::instance().insTopic, &insData_);

    for (uint8_t i = 1; i <= 4; i++) {
        swervePosPid_._[i - 1] = &swervePosPid[i - 1];
        swerveVelPid_._[i - 1] = &swerveVelPid[i - 1];
        swerveFilterInput_._[i - 1] = &swerveFilterInput[i - 1];
        swerveFilterOutput_._[i - 1] = &swerveFilterOutput[i - 1];
        wheelVelPid_._[i - 1] = &wheelVelPid[i - 1];

        InitConfig_s gm6020Config = {
            .pComHandle = reinterpret_cast<uint32_t *>(swerveCan[i - 1]),
            .comType = ComType_e::CAN,
            .workMode = WorkMode_e::QUAD_CURR,
            .offsetId = i,
            .txFreq = 100.0f,
            .posPID = nullptr,
            .velPID = nullptr,
            .isReverse = true,
        };
        motors_._[i + 3] = new DJIMOTOR::GM6020("GM6020", gm6020Config);
        motors_._[i + 3]->setZeroAng(SwerveZero._[i - 1]);

        InitConfig_s m3508Config = {
            .pComHandle = reinterpret_cast<uint32_t *>(wheelCan[i - 1]),
            .comType = ComType_e::CAN,
            .workMode = WorkMode_e::QUAD_CURR,
            .offsetId = i,
            .txFreq = 100.0f,
            .posPID = nullptr,
            .velPID = nullptr,
            .isReverse = true,
        };
        motors_._[i - 1] = new DJIMOTOR::M3508("M3508", m3508Config);
        motors_._[i - 1]->overrideReductionRatio(DJIMOTOR::M3508::XROLL_GEARBOX_RR);
    }

    this->stateFactory_.addState(FSMState_e::RUN, std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    this->stateFactory_.addState(FSMState_e::TEST, std::make_unique<TestState>(this));
    this->stateFactory_.addState(FSMState_e::SPIN, std::make_unique<SpinState>(this));
    this->stateFactory_.addState(FSMState_e::TANK, std::make_unique<TANKState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));

    if constexpr (APP_USE_POWERCTRL) {
        powerCtrl_ = new SwerveController(ChassisType_e::SWERVE, &cap_);
    }
}

void Swerve::stop()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::OFF);
    }
}

void Swerve::enter()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::ON);
    }
}

void Swerve::update(void *_param)
{
#if !APP_USE_COMM
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) == pdTRUE) {
    };

#else
    msg.state = static_cast<FSMState_e>(chassisCtrlRxPacket_.data().state);
    msg.vx = chassisCtrlRxPacket_.data().vx;
    msg.vy = chassisCtrlRxPacket_.data().vy;
    msg.wz = chassisCtrlRxPacket_.data().wz;
    isGimbalReset = chassisCtrlRxPacket_.data().gimbalReset;

    // sentryCmd.posture = static_cast<uint32_t>(chassisCtrlRxPacket_.data().sentryPosture);
    // referee->transmitter.sendData(0x0120, reinterpret_cast<uint8_t *>(&sentryCmd), sizeof(REFEREE::SentryCmd_s));
#endif

    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) == pdTRUE) {
    };

    loadDeltaYaw();
    loadRefereeData(_param);

    for (uint8_t i = 0; i < 4; i++) {
        wSpeed_._[i] = motors_._[i]->data().spdRadps;
        curDir._[i] = motors_._[i + 4]->posNorm();
        sCurVel._[i] = swerveFilterInput_._[i]->process(motors_._[i + 4]->data().spdRadps);
        //debug
        sCurTorq._[i] = motors_._[i + 4]->data().torq;
        wCurr[i] = motors_._[i]->data().torq;
    }

    //转为头向速度
    Speed_u v = forward(wSpeed_);
    curSpeed_.vx = v.vx * cosYaw_ + v.vy * sinYaw_;
    curSpeed_.vy = v.vy * cosYaw_ - v.vx * sinYaw_;
    curSpeed_.wz = v.wz;

    if constexpr (APP_USE_POWERCTRL) {
        cap_.capTask(powerCtrl_->chargeCmdPower, powerCtrl_->capEnable_, powerCtrl_->capCharge,
                     static_cast<uint16_t>(powerCtrl_->chassisSetPower));
    }

    this->stateFactory_.update();
}
//向上板发送裁判系统数据
void Swerve::loadRefereeData(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->refereeQueue), &refereeMsg_, 0) == pdTRUE) {
    };

    refereeTxPacket_.setData().gameTime = refereeMsg_.gameTime;
    refereeTxPacket_.setData().gameProgress = referee->receiver.getRefereeData().gameStatus.gameProgress;
    refereeTxPacket_.setData().HP = refereeMsg_.currentHP;
    refereeTxPacket_.setData().isPlay = refereeMsg_.isPlay;
    refereeTxPacket_.setData().robotId = refereeMsg_.robotId;

    BulletSpdTxPacket_.setData().shootSpeed = refereeMsg_.bulletSpeed;

    chassisCurPower_ = refereeMsg_.chassisPowerLimit;
    chassisPowerBuffer_ = refereeMsg_.chassisPowerBuffer;
}

/**
 * @brief 轮速得车速
 * @see 模型：https://www.zxytql.top/algorithm/Swerveing_wheel
 * @param _wSpeed 
 * @return Speed_u 
 */
Speed_u Swerve::forward(const WheelsSpeed_u &_wSpeed)
{
    Speed_u speed;

    ///<这些数组索引与模型对应即，0：LF，1：LB，2：RB，3：RF
    float wheelComVel[4], sinAng[4], cosAng[4];

    float rX = W_FRONT_REAR / 2.f; ///<前后轮距的一半
    float rY = W_LEFT_RIGHT / 2.f; ///<左右轮距的一半
    float carRadius = sqrtf((rX * rX) + (rY * rY));

    wheelComVel[0] = -_wSpeed.M_LF * W_RADIUS; //m/s
    wheelComVel[1] = -_wSpeed.M_LB * W_RADIUS; //m/s
    wheelComVel[2] = _wSpeed.M_RB * W_RADIUS;  //m/s
    wheelComVel[3] = _wSpeed.M_RF * W_RADIUS;  //m/s

    sinAng[0] = sin(curDir.LF);
    cosAng[0] = cos(curDir.LF);

    sinAng[1] = sin(curDir.LB);
    cosAng[1] = cos(curDir.LB);

    sinAng[2] = sin(curDir.RB);
    cosAng[2] = cos(curDir.RB);

    sinAng[3] = sin(curDir.RF);
    cosAng[3] = cos(curDir.RF);

    speed.vx = (wheelComVel[0] * cosAng[0] + wheelComVel[1] * cosAng[1] + wheelComVel[2] * cosAng[2] +
                wheelComVel[3] * cosAng[3]) /
               4;
    speed.vy = (wheelComVel[0] * sinAng[0] + wheelComVel[1] * sinAng[1] + wheelComVel[2] * sinAng[2] +
                wheelComVel[3] * sinAng[3]) /
               4;
    speed.wz =
            (wheelComVel[0] * (rX * sinAng[0] - rY * cosAng[0]) + wheelComVel[1] * (-rX * sinAng[1] - rY * cosAng[1]) +
             wheelComVel[2] * (-rX * sinAng[2] + rY * cosAng[2]) + wheelComVel[3] * (rX * sinAng[3] + rY * cosAng[3])) /
            powf(carRadius, 2) / 4;

    return speed;
}

/**
 * @brief 车速得轮速
 * @see 模型：https://www.zxytql.top/Algorithm/Steering_Wheel
 * @param _speed 
 * @return WheelsSpeed_u 
 */
WheelsSpeed_u Swerve::reverse(const Speed_u &_speed)
{
    WheelsSpeed_u refWheels;
    ///<这些数组索引与模型对应即，0：LF，1：LB，2：RB，3：RF
    float wheelVx[4], wheelVy[4], wheelComVel[4];

    float rX = W_FRONT_REAR / 2.f; ///<前后轮距的一半
    float rY = W_LEFT_RIGHT / 2.f; ///<左右轮距的一半

    //LF:0
    wheelVx[0] = _speed.vx - _speed.wz * rY;
    wheelVy[0] = _speed.vy + _speed.wz * rX;
    arm_sqrt_f32((wheelVx[0] * wheelVx[0]) + (wheelVy[0] * wheelVy[0]), &wheelComVel[0]);
    arm_atan2_f32(wheelVy[0], wheelVx[0], &expDir.LF);
    //LB:1
    wheelVx[1] = _speed.vx - _speed.wz * rY;
    wheelVy[1] = _speed.vy - _speed.wz * rX;
    arm_sqrt_f32((wheelVx[1] * wheelVx[1]) + (wheelVy[1] * wheelVy[1]), &wheelComVel[1]);
    arm_atan2_f32(wheelVy[1], wheelVx[1], &expDir.LB);
    //RB:2
    wheelVx[2] = _speed.vx + _speed.wz * rY;
    wheelVy[2] = _speed.vy - _speed.wz * rX;
    arm_sqrt_f32((wheelVx[2] * wheelVx[2]) + (wheelVy[2] * wheelVy[2]), &wheelComVel[2]);
    arm_atan2_f32(wheelVy[2], wheelVx[2], &expDir.RB);
    //RF:3
    wheelVx[3] = _speed.vx + _speed.wz * rY;
    wheelVy[3] = _speed.vy + _speed.wz * rX;
    arm_sqrt_f32((wheelVx[3] * wheelVx[3]) + (wheelVy[3] * wheelVy[3]), &wheelComVel[3]);
    arm_atan2_f32(wheelVy[3], wheelVx[3], &expDir.RF); //反正切返回(-pi,pi)
    for (float &i : expDir._) {
        if (i < 0) {
            i += 2 * F_PI; //[0,2pi]
        }
    }
    //传rad/s
    refWheels.M_LF = -wheelComVel[0] / W_RADIUS;
    refWheels.M_LB = -wheelComVel[1] / W_RADIUS;
    refWheels.M_RB = wheelComVel[2] / W_RADIUS;
    refWheels.M_RF = wheelComVel[3] / W_RADIUS;

    return refWheels;
}

/**
 * @brief 控制电机
 * @note 模型中逆时针角度增大，实际6020id端在上,顺时针编码值增大。
 * @param _refSpeed 
 */
float iq[8] = {};
float rawiq[8] = {};
float powerCmd[8] = {}; //功率控制
void Swerve::ctrl(const Speed_u &_refSpeed)
{
    Swerve_u<float> tempDir;
    WheelsSpeed_u refWSpeed = {};
    float expRads[8] = {}; //功率控制
    float sDebugVel[4] = {};

    float vx = std::fmax(-MAX_VX_SPEED, std::fmin(_refSpeed.vx, MAX_VX_SPEED));
    float vy = std::fmax(-MAX_VY_SPEED, std::fmin(_refSpeed.vy, MAX_VY_SPEED));
    float wz = std::fmax(-MAX_WZ_SPEED, std::fmin(_refSpeed.wz, MAX_WZ_SPEED));

    Speed_u refSpeed = { .vx = (vx * this->cosYaw_) - (vy * this->sinYaw_),
                         .vy = (vy * this->cosYaw_) + (vx * this->sinYaw_),
                         .wz = wz };

    if (vx != 0 || vy != 0 || wz != 0)
        refWSpeed = reverse(refSpeed);

    for (uint8_t i = 0; i < 4; i++) {
        //6020
        tempDir._[i] = expDir._[i];
        //先决定正反转,要6020转角最小，最好一个对角线上的3508速度直接反向。
        if (fabs(getMinorArc(expDir._[i], curDir._[i])) >= F_PI / 2) {
            refWSpeed._[i] = -refWSpeed._[i];
            tempDir._[i] -= F_PI; //[0,pi]
        }

        refWSpeed_ = refWSpeed; //debug

        sDel[i] = getMinorArc(tempDir._[i], curDir._[i]);
        sExpVel[i] = swervePosPid_._[i]->calc(sDel[i], 0);
        sTorq[i] = swerveVelPid_._[i]->calc(sExpVel[i], sCurVel._[i]);

        powerCmd[i + 4] = sTorq[i];
        // motors_.S_RF->cmdTorq(0);
        // motors_.S_LF->cmdTorq(0);
        // motors_.S_LB->cmdTorq(0);
        // motors_.S_RB->cmdTorq(0);

        //3508
        wTorq[i] = wheelVelPid_._[i]->calc(refWSpeed._[i], wSpeed_._[i]);
        powerCmd[i] = wTorq[i];
        // powerCmd[i] = 0;

        // motors_._[i]->cmdTorq(0); //不动
        // motors_.M_RF->cmdTorq(0);
        // motors_.M_LF->cmdTorq(0);
        // motors_.M_LB->cmdTorq(0);
        // motors_.M_RB->cmdTorq(0);
        expRads[i] = refWSpeed._[i];
        expRads[i + 4] = sExpVel[i];
        rawiq[i] = powerCmd[i];
        rawiq[i + 4] = powerCmd[i + 4];
        sDebugVel[i] = motors_._[i + 4]->vel();
    }

    std::vector<float> torq;
    torq = powerCtrl_->powerCtrl(expRads, motors_._, powerCmd, refereeMsg_, sDebugVel);
    for (uint8_t i = 0; i < 8; i++) {
        iq[i] = torq[i];
#if APP_USE_POWERCTRL
        motors_._[i]->cmdTorq(torq[i]); //功率控制
                                        // motors_._[i]->cmdTorq(powerCmd[i]);
#else
        motors_._[i]->cmdTorq(powerCmd[i]);
#endif
    }
}

void Swerve::loadDeltaYaw()
{
    // Master Board method
    insSub_->receive();
    deltaYaw = getMinorArc(chassisCtrlRxPacket_.data().deltaYaw, 0.f);
    cosYaw_ = arm_cos_f32(deltaYaw);
    sinYaw_ = arm_sin_f32(deltaYaw);
};
