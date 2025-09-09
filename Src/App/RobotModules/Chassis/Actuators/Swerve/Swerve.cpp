#include "Swerve.hpp"
#include "sdkconfig.h"
#include "Bsp_can.hpp"
#include "PidBasic.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include "GM6020.hpp"
#include "M3508.hpp"

#include "QuadricycleController.hpp"

#include "./State/StopState.hpp"
#include "./State/RunState.hpp"
#include "Cmd.hpp"

extern canHandle HCAN1;
extern canHandle HCAN2;

extern Cmd cmd;

using namespace CHASSIS;
using namespace PINYMOTOR;

constexpr float F_PI = std::numbers::pi_v<float>;

//以下数组均可调，未测试
static PositonalPid swervePosPid[4] = {
    PositonalPid(20.f, 0.001f, 0.f, 1.f / 1000.f, 0.f, 35.f, 0.f),
    PositonalPid(18.f, 0.001f, 0.f, 1.f / 1000.f, 0.f, 35.f, 0.f),
    PositonalPid(20.f, 0.001f, 0.f, 1.f / 1000.f, 0.f, 35.f, 0.f),
    PositonalPid(20.f, 0.002f, 0.f, 1.f / 1000.f, 0.f, 35.f, 0.f)
};
static PositonalPid swerveVelPid[4] = {
    PositonalPid(0.01f, 0.001f, 0.f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositonalPid(0.01f, 0.001f, 0.f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositonalPid(0.01f, 0.001f, 0.f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    PositonalPid(0.01f, 0.0015f, 0.f, 1.f / 1000.f, 0.3f, 1.2f, 0.f)
};
//此滤波并没有调参
static FILTER::LPF swerveFilterOutput[4]{ FILTER::LPF(1000.f, 4.f),
                                          FILTER::LPF(1000.f, 4.f),
                                          FILTER::LPF(1000.f, 4.f),
                                          FILTER::LPF(1000.f, 4.f) };

Swerve::Swerve()
{
    for (uint8_t i = 1; i <= 4; i++) {
        //输出滤波初始化
        swerveFilterOutput_._[i - 1] = &swerveFilterOutput[i - 1];
        //舵轮pid初始化
        swervePosPid_._[i - 1] = &swervePosPid[i - 1];
        swerveVelPid_._[i - 1] = &swerveVelPid[i - 1];

        InitConfig_s gm6020Config = {
            .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
            .comType = ComType_e::CAN,
            .workMode = WorkMode_e::QUAD_CURR,
            .offsetId = i,
            .txFreq = 100.0f,
            .posPID = nullptr,
            .velPID = nullptr,
            .torqPID = nullptr,
            .isReverse = true,
        };
        motors_._[i + 3] = new DJIMOTOR::GM6020("GM6020", gm6020Config);
        motors_._[i + 3]->setZeroAng(SwerveZero[i - 1]);

        InitConfig_s m3508Config = {
            .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
            .comType = ComType_e::CAN,
            .workMode = WorkMode_e::QUAD_CURR,
            .offsetId = i,
            .txFreq = 100.0f,
            .posPID = nullptr,
            .velPID = new IncrementalPid(0.5f, 0.005f, 0.f, 4.f, 0.f),
            .torqPID = nullptr
        };
        motors_._[i - 1] = new DJIMOTOR::M3508("M3508", m3508Config);
    }

    this->stateFactory_.addState(FSMState_e::RUN,
                                 std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::STOP,
                                 std::make_unique<StopState>(this));
    this->stateFactory_.init(
            this->stateFactory_.getNextState(FSMState_e::STOP));


    if constexpr (APP_USE_POWERCTRL) {
        //
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


void Swerve::update()
{
    xQueueReceive(cmd.getMsgBus()->chassisQueue, &msg, 0);

    for (uint8_t i = 0; i < 4; i++) {
        wSpeed_._[i] = motors_._[i]->data().spdRpm;
        curDir._[i] = motors_._[i + 4]->data().ang;
    }
    this->curSpeed_ = forward(wSpeed_);
    this->stateFactory_.update();
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
    float wheelVx[4], wheelVy[4], wheelComVel[4], sinAng[4], cosAng[4];

    float rX = W_FRONT_REAR / 2.f; ///<前后轮距的一半
    float rY = W_LEFT_RIGHT / 2.f; ///<左右轮距的一半
    float carRadius = sqrtf((rX * rX) + (rY * rY));

    wheelComVel[0] = -_wSpeed.M_LF * W_CIRCUMFERENCE / 60; //m/s
    wheelComVel[1] = _wSpeed.M_LB * W_CIRCUMFERENCE / 60;  //m/s
    wheelComVel[2] = _wSpeed.M_RB * W_CIRCUMFERENCE / 60;  //m/s
    wheelComVel[3] = -_wSpeed.M_RF * W_CIRCUMFERENCE / 60; //m/s

    sinAng[0] = sin(curDir.DirAng_LF);
    cosAng[0] = cos(curDir.DirAng_LF);

    sinAng[1] = sin(curDir.DirAng_LB);
    cosAng[1] = cos(curDir.DirAng_LB);

    sinAng[2] = sin(curDir.DirAng_RB);
    cosAng[2] = cos(curDir.DirAng_RB);

    sinAng[3] = sin(curDir.DirAng_RF);
    cosAng[3] = cos(curDir.DirAng_RF);


    speed.vx = (wheelComVel[0] * cosAng[0] + wheelComVel[1] * cosAng[1] +
                wheelComVel[2] * cosAng[2] + wheelComVel[3] * cosAng[3]) /
               4;
    speed.vy = (wheelComVel[0] * sinAng[0] + wheelComVel[1] * sinAng[1] +
                wheelComVel[2] * sinAng[2] + wheelComVel[3] * sinAng[3]) /
               4;
    speed.wz = (wheelComVel[0] * (rX * sinAng[0] - rY * cosAng[0]) +
                wheelComVel[1] * (-rX * sinAng[1] - rY * cosAng[1]) +
                wheelComVel[2] * (-rX * sinAng[2] + rY * cosAng[2]) +
                wheelComVel[3] * (rX * sinAng[3] + rY * cosAng[3])) /
               powf(carRadius, 2) / 4;

    return speed;
}

/**
 * @brief 车速得轮速
 * @see 模型：https://www.zxytql.top/algorithm/Swerveing_wheel
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
    float carRadius = sqrtf((rX * rX) + (rY * rY));

    //LF:0
    wheelVx[0] = _speed.vx - _speed.wz * rY;
    wheelVy[0] = _speed.vy + _speed.wz * rX;
    wheelComVel[0] =
            sqrtf((wheelVx[0] * wheelVx[0]) + (wheelVy[0] * wheelVy[0]));

    expDir.DirAng_LF = atan2(wheelVy[0], wheelVx[0]);

    //LB:1
    wheelVx[1] = _speed.vx - _speed.wz * rY;
    wheelVy[1] = _speed.vy - _speed.wz * rX;
    wheelComVel[1] =
            sqrtf((wheelVx[1] * wheelVx[1]) + (wheelVy[1] * wheelVy[1]));

    expDir.DirAng_LB = atan2(wheelVy[1], wheelVx[1]);

    //RB:2
    wheelVx[2] = _speed.vx + _speed.wz * rY;
    wheelVy[2] = _speed.vy - _speed.wz * rX;
    wheelComVel[2] =
            sqrtf((wheelVx[2] * wheelVx[2]) + (wheelVy[2] * wheelVy[2]));

    expDir.DirAng_RB = atan2(wheelVy[2], wheelVx[2]);

    //RF:3
    wheelVx[3] = _speed.vx + _speed.wz * rY;
    wheelVy[3] = _speed.vy + _speed.wz * rX;
    wheelComVel[3] =
            sqrtf((wheelVx[3] * wheelVx[3]) + (wheelVy[3] * wheelVy[3]));

    expDir.DirAng_RF = atan2(wheelVy[3], wheelVx[3]); //反正切返回(-pi,pi)

    for (float &i : expDir._) {
        if (i < 0) {
            i += 2 * F_PI; //[0,2pi]
        }
    }

    refWheels.M_LF = -wheelComVel[0] / W_CIRCUMFERENCE * 60;
    refWheels.M_LB = wheelComVel[1] / W_CIRCUMFERENCE * 60;
    refWheels.M_RB = wheelComVel[2] / W_CIRCUMFERENCE * 60;
    refWheels.M_RF = -wheelComVel[3] / W_CIRCUMFERENCE * 60;

    return refWheels;
}

/**
 * @brief 控制电机
 * @note 模型中逆时针角度增大，实际6020id端在上,顺时针编码值增大。
 * @param _refSpeed 
 */
void Swerve::ctrl(const Speed_u &_refSpeed)
{
    SwerveDirectionAngle_u tempDir;
    float del[4], vel[4], torq[4];

    // float cosYaw = arm_cos_f32(deltaYaw_);
    // float sinYaw = arm_sin_f32(deltaYaw_);
    // Speed_u refSpeed = {
    //     .vx = (_refSpeed.vx * cosYaw) - (_refSpeed.vy * sinYaw),
    //     .vy = (_refSpeed.vy * cosYaw) + (_refSpeed.vx * sinYaw),
    //     .wz = _refSpeed.wz
    // };

    WheelsSpeed_u refWSpeed = reverse(_refSpeed);

    for (uint8_t i = 0; i < 4; i++) {
        tempDir._[i] = expDir._[i];
        //先决定正反转,要6020转角最小，最好一个对角线上的3508速度直接反向。
        if (fabs(getMinorArc(expDir._[i], motors_._[i + 4]->posNorm())) >=
            F_PI / 2) {
            refWSpeed._[i] = -refWSpeed._[i];
            tempDir._[i] -= F_PI; //[0,pi]
        }
        tempDir._[i] < 0 ? 0 : tempDir._[i];

        del[i] = getMinorArc(tempDir._[i], motors_._[i + 4]->posNorm());
        vel[i] = swervePosPid_._[i]->calc(del[i], 0);
        torq[i] = swerveVelPid_._[i]->calc(vel[i], motors_._[i + 4]->vel());

        //6020
        if (_refSpeed.vx == 0 && _refSpeed.vy == 0 && _refSpeed.wz == 0) {
            motors_._[i + 4]->cmdTorq(0);
        } else {
            motors_._[i + 4]->cmdTorq(
                    swerveFilterOutput_._[i]->process(torq[i]));
        }
        //3508
        motors_._[i]->cmdVel(refWSpeed._[i]);
        // motors_._[i]->cmdVel(0); //不动
    }

    if constexpr (APP_USE_POWERCTRL) {
        // powerCtrl_->powerCtrl(refWSpeed._);
    }
}