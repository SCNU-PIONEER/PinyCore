#include "Swerve.hpp"
#include "Chassis.hpp"
#include "IMotor.hpp"
#include "MotorCommonMacros.hpp"
#include "Referee.hpp"
#include "RefereeProt.hpp"
#include "Soc.hpp"
#include "dsp/fast_math_functions.h"
#include "sdkconfig.h"
#include "PidBasic.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include "GM6020.hpp"
#include "M3508.hpp"
#include "TopicRouter.hpp"
#include "./State/StopState.hpp"
#include "./State/RunState.hpp"
#include "./State/SpinState.hpp"
#include "./State/TANKState.hpp"
#include "./State/FakeState.hpp"


extern canHandle HCAN1;
extern canHandle HCAN2;
extern canHandle HCAN3;

using namespace CHASSIS;
using namespace PINYMOTOR;

constexpr float F_PI = std::numbers::pi_v<float>;

//以下数组可以塞到类内。但不好用free master
//RF 0 ,LF 1 ,LB 2 ,RB 3
FILTER::LPF steerFilterInput[4]{
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 4.f),
};

#if !APP_USE_COMM && !APP_USE_POWERCTRL //无头骑士
PositionalPid steerPosPid[4] = {
    PositionalPid(20.f, 15.f, 0.4f, 1.f / 1000.f, 10.f, 35.f, 0.f),
    PositionalPid(25.f, 20.f, 0.4f, 1.f / 1000.f, 10.f, 35.f, 0.f),
    PositionalPid(25.f, 15.f, 0.4f, 1.f / 1000.f, 10.f, 35.f, 0.f),
    PositionalPid(25.f, 15.f, 0.4f, 1.f / 1000.f, 10.f, 35.f, 0.f),
};
PositionalPid steerVelPid[4] = {
    PositionalPid(0.2f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositionalPid(0.2f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositionalPid(0.2f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositionalPid(0.2f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
};
#elif APP_USE_POWERCTRL
#include "SwerveController.hpp"
PositionalPid steerPosPid[4] = {
    PositionalPid(45.f, 0.f, 0.45f, 1.f / 1000.f, 10.f, 35.f, 0.f),
    PositionalPid(45.f, 0.f, 0.5f, 1.f / 1000.f, 10.f, 35.f, 0.f),
    PositionalPid(45.f, 0.f, 0.3f, 1.f / 1000.f, 10.f, 35.f, 0.f),
    PositionalPid(30.f, 0.f, 0.2f, 1.f / 1000.f, 10.f, 35.f, 0.f),
};
PositionalPid steerVelPid[4] = {
    PositionalPid(0.05f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.f, 0.f),
    PositionalPid(0.05f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.f, 0.f),
    PositionalPid(0.04f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.f, 0.f),
    PositionalPid(0.05f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.f, 0.f),
};
#else
PositionalPid steerPosPid[4] = {
    PositionalPid(15.f, 0.f, 0.2f, 1.f / 1000.f, 10.f, 35.f, 0.f),
    PositionalPid(15.f, 0.f, 0.1f, 1.f / 1000.f, 10.f, 35.f, 0.f),
    PositionalPid(30.f, 10.f, 0.25f, 1.f / 1000.f, 10.f, 35.f, 0.f),
    PositionalPid(15.f, 0.f, 0.02f, 1.f / 1000.f, 10.f, 35.f, 0.f),
};
PositionalPid steerVelPid[4] = {
    PositionalPid(0.1f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositionalPid(0.1f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositionalPid(0.1f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositionalPid(0.1f, 0.0f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
};
#endif

PositionalPid sBumpVelPid[4] = {
    PositionalPid(0.05f, 0.001f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositionalPid(0.05f, 0.001f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositionalPid(0.04f, 0.001f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositionalPid(0.05f, 0.001f, 0.0f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
};

PositionalPid wheelVelPid[4] = {
    PositionalPid(0.45f, 0.000f, 0.001f, 1.f / 1000.f, 1.f, 7.f, 0.1f),
    PositionalPid(0.45f, 0.000f, 0.001f, 1.f / 1000.f, 1.f, 7.f, 0.1f),
    PositionalPid(0.45f, 0.000f, 0.001f, 1.f / 1000.f, 1.f, 7.f, 0.1f),
    PositionalPid(0.45f, 0.000f, 0.001f, 1.f / 1000.f, 1.f, 7.f, 0.1f),
};

PositionalPid wBumpVelPid[4] = {
    PositionalPid(0.6f, 0.5f, 0.001f, 1.f / 1000.f, 3.8f, 7.f, 0.1f),
    PositionalPid(0.6f, 0.5f, 0.001f, 1.f / 1000.f, 3.8f, 7.f, 0.1f),
    PositionalPid(0.6f, 0.5f, 0.001f, 1.f / 1000.f, 3.8f, 7.f, 0.1f),
    PositionalPid(0.6f, 0.5f, 0.001f, 1.f / 1000.f, 3.8f, 7.f, 0.1f),
};

Swerve::Swerve()
{
    insSub_ = new Subscriber<INS_SYS::INSData_s>(4, &TopicRouter::instance().insTopic, &insData_);

    for (uint8_t i = 1; i <= 4; i++) {
        steerFilterInput_._[i - 1] = &steerFilterInput[i - 1];
        steerPosPid_._[i - 1] = &steerPosPid[i - 1];
        steerVelPid_._[i - 1] = &steerVelPid[i - 1];
        sBumpVelPid_._[i - 1] = &sBumpVelPid[i - 1];

        wheelVelPid_._[i - 1] = &wheelVelPid[i - 1];
        wBumpVelPid_._[i - 1] = &wBumpVelPid[i - 1];

        InitConfig_s gm6020Config = {
            .pComHandle = reinterpret_cast<uint32_t *>(steerCan._[i - 1]),
            .comType = ComType_e::CAN,
            .workMode = WorkMode_e::QUAD_CURR,
            .offsetId = i,
            .txFreq = STEER_MOTOR_FREQ,
            .posPID = nullptr,
            .velPID = nullptr,
            .isReverse = true,
        };
        steerMotor_._[i - 1] = new DJIMOTOR::GM6020("GM6020", gm6020Config);
        steerMotor_._[i - 1]->setZeroAng(steerZero._[i - 1]);

        InitConfig_s m3508Config = {
            .pComHandle = reinterpret_cast<uint32_t *>(wheelCan._[i - 1]),
            .comType = ComType_e::CAN,
            .workMode = WorkMode_e::QUAD_CURR,
            .offsetId = i,
            .txFreq = WHEEL_MOTOR_FREQ,
            .posPID = nullptr,
            .velPID = nullptr,
            .isReverse = true,
        };
        wheelMotor_._[i - 1] = new DJIMOTOR::M3508("M3508", m3508Config);
        wheelMotor_._[i - 1]->overrideReductionRatio(DJIMOTOR::M3508::XROLL_GEARBOX_RR);
    }

    this->stateFactory_.addState(FSMState_e::RUN, std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    this->stateFactory_.addState(FSMState_e::SPIN, std::make_unique<SpinState>(this));
    this->stateFactory_.addState(FSMState_e::TANK, std::make_unique<TANKState>(this));
    this->stateFactory_.addState(FSMState_e::FAKE, std::make_unique<FakeState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));

#if APP_USE_POWERCTRL
    powerCtrl_ = new SwerveController(ChassisType_e::SWERVE, &cap_);
#endif
#if EXTENSION_SUPERCAP
    cap_.limitCharge();
#endif
}

void Swerve::stop()
{
    for (uint8_t i = 0; i < 4; i++) {
        steerMotor_._[i]->cmd(MotorCmdType_e::OFF);
        wheelMotor_._[i]->cmd(MotorCmdType_e::OFF);
    }
}

void Swerve::enter()
{
    for (uint8_t i = 0; i < 4; i++) {
        steerMotor_._[i]->cmd(MotorCmdType_e::ON);
        wheelMotor_._[i]->cmd(MotorCmdType_e::ON);
    }
}

void Swerve::update(void *_param)
{
#if !APP_USE_COMM
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) == pdTRUE) {
    };
    // sentryCmd.posture = static_cast<uint32_t>(SentryPosture_e::MOVE);
    isGimbalReset = true;
    // deltaYaw += msg.wz * 0.001f;
#else
    msg.state = static_cast<FSMState_e>(chassisCtrlRxPacket_.data().state);
    msg.vx = chassisCtrlRxPacket_.data().vx;
    msg.vy = chassisCtrlRxPacket_.data().vy;
    msg.wz = chassisCtrlRxPacket_.data().wz;
    isGimbalReset = chassisCtrlRxPacket_.data().gimbalReset;
    if (msg.state == FSMState_e::FAKE) {
        isGimbalReset = true;
    }
#endif

    refereeData_ = referee->receiver.getRefereeData();
    sentryInfo_ = refereeData_.sentryInfo;

    loadDeltaYaw();
    loadRefereeData(_param);
    sendDataToReferee();
    sendDecesionData();

    for (uint8_t i = 0; i < 4; i++) {
        curWSpeed_._[i] = wheelMotor_._[i]->data().spdRadps;
        curDir._[i] = steerMotor_._[i]->posNorm();
        sCurVel._[i] = steerFilterInput_._[i]->process(steerMotor_._[i]->data().spdRadps);
        sCurTorq._[i] = steerMotor_._[i]->data().torq;

        debug_.wCurr._[i] = wheelMotor_._[i]->data().curr;
    }

    //转为头向速度
    Speed_u v = forward(curWSpeed_);
    curSpeed_.vx = v.vx * cosYaw_ + v.vy * sinYaw_;
    curSpeed_.vy = v.vy * cosYaw_ - v.vx * sinYaw_;
    curSpeed_.wz = v.wz;

    if (refereeMsg_.chassisPowerLimit < 0 || refereeMsg_.chassisPowerLimit > 200) {
        refereeMsg_.chassisPowerLimit = 201;
    }

#if EXTENSION_SUPERCAP
    // float capRealRatio = cap_.getCapData().capEnergyRatio;
    cap_.setChargelimitRatio(0.95f);
    (refereeMsg_.currentHP != 0) ? cap_.enable() : cap_.disable();
#if EXTENSION_REFEREE
    cap_.setChargePowerLimit(refereeMsg_.chassisPowerLimit);
    cap_.setChargeEnergySlack(refereeMsg_.chassisPowerBuffer);
#if APP_USE_POWERCTRL
    capCmdRatio_ = (chassisCtrlRxPacket_.data().useSuperCap && refereeMsg_.currentHP != 0) ? 0.2f : 0.9f;
    powerCtrl_->capCmdRatio_ = capCmdRatio_;
#endif
#endif
    cap_.task();
#endif

    this->stateFactory_.update();
}
//向上板发送裁判系统数据
void Swerve::loadRefereeData(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->refereeQueue), &refereeMsg_, 0) == pdTRUE) {
    };

    refereeTxPacket_.setData().gameTime = refereeMsg_.gameTime;
    refereeTxPacket_.setData().gameProgress = refereeData_.gameStatus.gameProgress;
    refereeTxPacket_.setData().HP = refereeMsg_.currentHP;
    refereeTxPacket_.setData().isPlay = refereeMsg_.isPlay;
    refereeTxPacket_.setData().robotId = refereeMsg_.robotId;

    BulletSpdTxPacket_.setData().shootSpeed = refereeMsg_.bulletSpeed;
    BulletSpdTxPacket_.setData().bulletFreq = refereeData_.shootData.bulletFreq;

    debug_.chassisCurPower = refereeMsg_.chassisPowerLimit;
    debug_.chassisPowerBuffer = refereeMsg_.chassisPowerBuffer;
}

/*
    uint16_t dataCmdId = 0x0120;    //需为开放的子内容 ID
    uint16_t senderId;              //需与自身 ID 匹配,机器人ID
    uint16_t receiverId = 0x8080;   //裁判系统服务器(用于哨兵和雷达自主决策指令)
*/
void Swerve::sendDataToReferee()
{
    TickType_t cnt = xTaskGetTickCount();
    sentryCmdData_.dataCmdId = 0x0120;
    sentryCmdData_.senderId = refereeMsg_.robotId;
    sentryCmdData_.receiverId = 0x8080;
    sentryCmdData_.userdata = sentryCmdRxPacket_.data().midSentryCmd;

    if (fabs(refereeMsg_.currentHP) != 0) {
        sentryCmdData_.userdata.reviveConfirm = 0;
        sentryCmdData_.userdata.instantReviveConfirm = 0;
    } else {
        sentryCmdData_.userdata.reviveConfirm = 1;
        // sentryCmdData_.userdata.instantReviveConfirm = 1;
    }

    if (cnt % 100 == 0) { //10HZ裁判系统发送数据
        referee->transmitter.sendData(0x0301, reinterpret_cast<uint8_t *>(&sentryCmdData_), sizeof(sentryCmdData_));
    }
}

void Swerve::sendDecesionData()
{
    decesionTxPacket1_.setData().currentChassisEnergy = refereeData_.buff.remainingEnergy;
    decesionTxPacket1_.setData().allyBaseHP = refereeData_.gameRobotHP.allyBaseHP;
    decesionTxPacket1_.setData().allyOutpostHP = refereeData_.gameRobotHP.allyOutpostHP;
    sentryInfoTxPacket_.setData().midSentryInfo = sentryInfo_;
    sentryInfoTxPacket_.setData().hurtHpDeductionReason = refereeData_.robotHurt.reason;
}

/**
 * @brief 轮速得车速
 * @see 模型：https://www.zxytql.top/algorithm/Swerveing_wheel
 * @param _wSpeed 
 * @return Speed_u 
 */
Speed_u Swerve::forward(const Swerve_u<float> &_wSpeed)
{
    Speed_u speed;
    ///<这些数组索引与模型对应即,0:LF,1:LB,2:RB,3:RF
    float wheelComVel[4], sinAng[4], cosAng[4];

    float rX = W_FRONT_REAR / 2.f; ///<前后轮距的一半
    float rY = W_LEFT_RIGHT / 2.f; ///<左右轮距的一半
    float carRadius = sqrtf((rX * rX) + (rY * rY));

    wheelComVel[0] = -_wSpeed.LF * W_RADIUS; //m/s
    wheelComVel[1] = -_wSpeed.LB * W_RADIUS; //m/s
    wheelComVel[2] = _wSpeed.RB * W_RADIUS;  //m/s
    wheelComVel[3] = _wSpeed.RF * W_RADIUS;  //m/s

    sinAng[0] = arm_sin_f32(curDir.LF);
    cosAng[0] = arm_cos_f32(curDir.LF);

    sinAng[1] = arm_sin_f32(curDir.LB);
    cosAng[1] = arm_cos_f32(curDir.LB);

    sinAng[2] = arm_sin_f32(curDir.RB);
    cosAng[2] = arm_cos_f32(curDir.RB);

    sinAng[3] = arm_sin_f32(curDir.RF);
    cosAng[3] = arm_cos_f32(curDir.RF);

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
Swerve_u<float> Swerve::reverse(const Speed_u &_speed)
{
    Swerve_u<float> refWheels;
    ///<这些数组索引与模型对应即,0:LF,1:LB,2:RB,3:RF
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
    refWheels.LF = -wheelComVel[0] / W_RADIUS;
    refWheels.LB = -wheelComVel[1] / W_RADIUS;
    refWheels.RB = wheelComVel[2] / W_RADIUS;
    refWheels.RF = wheelComVel[3] / W_RADIUS;

    return refWheels;
}

/**
 * @brief 控制电机
 * @note 模型中逆时针角度增大，实际6020id端在上,顺时针编码值增大。
 * @param _refSpeed 
 */
void Swerve::ctrl(const Speed_u &_refSpeed)
{
    Swerve_u<float> tempDir, refWSpeed = {};
    Swerve_u<float> sDel, sExpVel, sExpTorq, wExpTorq;
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
        //先决定正反转,要6020转角最小，对角线上的3508速度直接反向。
        if (fabs(getMinorArc(expDir._[i], curDir._[i])) >= F_PI / 2) {
            refWSpeed._[i] = -refWSpeed._[i];
            tempDir._[i] -= F_PI; //[0,pi]
        }
        sDel._[i] = getMinorArc(tempDir._[i], curDir._[i]);
        sExpVel._[i] = steerPosPid_._[i]->calc(sDel._[i], 0);
        sExpTorq._[i] = steerVelPid_._[i]->calc(sExpVel._[i], sCurVel._[i]);
        //3508
        expWSpeed_._[i] = refWSpeed._[i];
        wExpTorq._[i] = wheelVelPid_._[i]->calc(expWSpeed_._[i], curWSpeed_._[i]);

        if (sentryCmdRxPacket_.data().isBumpArea) {
            sExpTorq._[i] = sBumpVelPid_._[i]->calc(sExpVel._[i], sCurVel._[i]);
            wExpTorq._[i] = wBumpVelPid_._[i]->calc(expWSpeed_._[i], curWSpeed_._[i]);
        }
    }

#if APP_USE_POWERCTRL
    powerCtrl(sExpVel, sExpTorq, expWSpeed_, wExpTorq);
#else
    motorCtrl(sExpTorq, wExpTorq);
#endif

    debug_.sExpVel = sExpVel;
    debug_.sExpTorq = sExpTorq;
    debug_.wExpTorq = wExpTorq;
}


#if APP_USE_POWERCTRL
Swerve_u<float> sPowerTorq = {};
Swerve_u<float> wPowerTorq = {};
void Swerve::powerCtrl(Swerve_u<float> _sExpSpd, Swerve_u<float> _sExpTorq, Swerve_u<float> _wExpSpd,
                       Swerve_u<float> _wExpTorq)
{
    SwervePower_u<float> expRads = {
        .W_RF = _wExpSpd.RF,
        .W_LF = _wExpSpd.LF,
        .W_LB = _wExpSpd.LB,
        .W_RB = _wExpSpd.RB,
        .S_RF = _sExpSpd.RF,
        .S_LF = _sExpSpd.LF,
        .S_LB = _sExpSpd.LB,
        .S_RB = _sExpSpd.RB,
    };
    SwervePower_u<float> expTorq = {
        .W_RF = _wExpTorq.RF,
        .W_LF = _wExpTorq.LF,
        .W_LB = _wExpTorq.LB,
        .W_RB = _wExpTorq.RB,
        .S_RF = _sExpTorq.RF,
        .S_LF = _sExpTorq.LF,
        .S_LB = _sExpTorq.LB,
        .S_RB = _sExpTorq.RB,
    };
    SwervePower_u<PINYMOTOR::IMotor *> motors = {
        .W_RF = wheelMotor_.RF,
        .W_LF = wheelMotor_.LF,
        .W_LB = wheelMotor_.LB,
        .W_RB = wheelMotor_.RB,
        .S_RF = steerMotor_.RF,
        .S_LF = steerMotor_.LF,
        .S_LB = steerMotor_.LB,
        .S_RB = steerMotor_.RB,
    };

    std::vector<float> torq;
    torq = powerCtrl_->powerCtrl(expRads._, motors._, expTorq._, refereeMsg_, sCurVel._);
    for (uint8_t i = 0; i < 4; i++) {
        wPowerTorq._[i] = torq[i];
        sPowerTorq._[i] = torq[i + 4];
    }
    // motorCtrl(sPowerTorq, wPowerTorq);
    motorCtrl(_sExpTorq, _wExpTorq);
}
#endif

//让你的舵轮像云台一样旋转
void Swerve::fakeGimbalCtrl(float _wz)
{
    Swerve_u<float> sTorq;
    _wz = clamp(_wz, -2.f, 2.f);
    for (float &i : sTorq._) {
        i = _wz / 2.f * 1.f;
    }

#if APP_USE_POWERCTRL
    powerCtrl({}, sTorq, {}, {});
    // motorCtrl(sTorq, Swerve_u<float>{});
#else
    motorCtrl(sTorq, Swerve_u<float>{});
#endif
    debug_.wExpTorq = {};
}

void Swerve::motorCtrl(Swerve_u<float> _sTorq, Swerve_u<float> _wTorq)
{
    for (uint8_t i = 0; i < 4; i++) {
        steerMotor_._[i]->cmdTorq(_sTorq._[i]);
        wheelMotor_._[i]->cmdTorq(_wTorq._[i]);
    }
    // steerMotor_.RF->cmdTorq(0);
    // steerMotor_.LF->cmdTorq(0);
    // steerMotor_.LB->cmdTorq(0);
    // steerMotor_.RB->cmdTorq(0);

    // wheelMotor_.RF->cmdTorq(0);
    // wheelMotor_.LF->cmdTorq(0);
    // wheelMotor_.LB->cmdTorq(0);
    // wheelMotor_.RB->cmdTorq(0);
}

void Swerve::loadDeltaYaw()
{
    // Master Board method
    insSub_->receive();
    deltaYaw = getMinorArc(chassisCtrlRxPacket_.data().deltaYaw, 0.f);
    cosYaw_ = arm_cos_f32(deltaYaw);
    sinYaw_ = arm_sin_f32(deltaYaw);
};
