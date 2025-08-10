#include <cmath>
#include <cstdint>
#include "Filter.hpp"
#include "Steer.hpp"
#include "M3508.hpp"
#include "GM6020.hpp"
#include "sdkconfig.h"
#include "Bsp_can.hpp"
#include "PidBasic.hpp"
#include <numbers>
#include "QuadricycleController.hpp"

extern canHandle HCAN1;
extern canHandle HCAN2;
extern canHandle HCAN3;

using namespace CHASSIS;
using namespace PINYMOTOR;

constexpr float F_PI = std::numbers::pi_v<float>;

Steer::Steer()
{
    for (uint8_t i = 1; i <= 4; i++) {
        InitConfig_s gm6020Config = {
            .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
            .comType = ComType_e::CAN,
            .workMode = WorkMode_e::QUAD_CURR,
            .offsetId = static_cast<uint8_t>(i + 4),
            .txFreq = 100.0f,
            .posPID = std::unique_ptr<PID>(
                    new positonalPid(0.2f, 0.005f, 0.f, 4.f, 0.f, 4.f, 0.f)),
            .velPID = std::unique_ptr<PID>(
                    new incrementalPid(0.2f, 0.005f, 0.f, 4.f, 0.f)),
            .torqPID = nullptr
        };
        motor_[i - 1] = new DJIMOTOR::GM6020("GM6020", std::move(gm6020Config));
    }
    for (uint8_t i = 1; i <= 4; i++) {
        InitConfig_s m3508Config = {
            .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
            .comType = ComType_e::CAN,
            .workMode = WorkMode_e::QUAD_CURR,
            .offsetId = i,
            .txFreq = 100.0f,
            .posPID = nullptr,
            .velPID = std::unique_ptr<PID>(
                    new incrementalPid(0.2f, 0.005f, 0.f, 4.f, 0.f)),
            .torqPID = nullptr
        };
        motor_[i + 3] = new DJIMOTOR::M3508("M3508", std::move(m3508Config));
    }

    if constexpr (USE_POWERCTRL) {
        powerCtrl_ = std::make_unique<QuadricycleController>(
                ChassisType_e::QUADRICYCLE);
    }
}

void Steer::stop()
{
    for (auto &i : motor_) {
        i->cmd(MotorCmdType_e::OFF);
    }
}

void Steer::enter()
{
    for (auto &i : motor_) {
        i->cmd(MotorCmdType_e::ON);
    }
}

void Steer::update()
{
    float tempdir[4];
    for (uint8_t i = 0; i < 8; i++) {
        wSpeed_._[i] = iir_filter_3(motor_[i]->data().spdRpm, i);
    }

    for (uint8_t i = 0; i < 4; i++) {
        tempdir[i] = iir_filter_3(motor_[i]->data().rawAng, i);
        ///<顺时针增大改为逆时针增大
        tempdir[i] = -tempdir[i] + 2 * F_PI;
        curDir._[i] = (tempdir[i] - SteerZero[i]);
    }

    this->curSpeed_ = forward(wSpeed_);
}

/**
 * @brief 轮速得车速
 * @see 模型：https://www.zxytql.top/algorithm/steering_wheel
 * @param _wSpeed 
 * @return Speed_u 
 */
Speed_u Steer::forward(const WheelsSpeed_u &_wSpeed)
{
    Speed_u speed;

    ///<这些数组索引与模型对应即，0：LF，1：LB，2：RB，3：RF
    float wheelVx[4], wheelVy[4], wheelComVel[4], sinAng[4], cosAng[4];

    float rX = W_FRONT_REAR / 2.f; ///<前后轮距的一半
    float rY = W_LEFT_RIGHT / 2.f; ///<左右轮距的一半
    float carRadius = sqrtf((rX * rX) + (rY * rY));

    for (uint8_t i = 0; i < 4; i++) {
        wheelComVel[i] *= W_CIRCUMFERENCE / 60;
    }
    sinAng[0] = sin(curDir.DirAng_LF);
    cosAng[0] = cos(curDir.DirAng_LF);

    sinAng[1] = sin(curDir.DirAng_LB);
    cosAng[1] = cos(curDir.DirAng_LB);

    sinAng[2] = sin(curDir.DirAng_RB);
    cosAng[2] = cos(curDir.DirAng_RB);

    sinAng[3] = sin(curDir.DirAng_RF);
    cosAng[3] = cos(curDir.DirAng_RF);


    speed.v_x = (wheelComVel[0] * cosAng[0] + wheelComVel[1] * cosAng[1] +
                 wheelComVel[2] * cosAng[2] + wheelComVel[3] * cosAng[3]) /
                4;
    speed.v_y = (wheelComVel[0] * sinAng[0] + wheelComVel[1] * sinAng[1] +
                 wheelComVel[2] * sinAng[2] + wheelComVel[3] * sinAng[3]) /
                4;
    speed.w_z = (wheelComVel[0] * (rX * sinAng[0] - rY * cosAng[0]) +
                 wheelComVel[1] * (-rX * sinAng[1] - rY * cosAng[1]) +
                 wheelComVel[2] * (-rX * sinAng[2] + rY * cosAng[2]) +
                 wheelComVel[3] * (rX * sinAng[3] + rY * cosAng[3])) /
                powf(carRadius, 2) / 4;

    return speed;
}

/**
 * @brief 车速得轮速
 * @see 模型：https://www.zxytql.top/algorithm/steering_wheel
 * @param _speed 
 * @return WheelsSpeed_u 
 */
WheelsSpeed_u Steer::reverse(const Speed_u &_speed)
{
    WheelsSpeed_u refWheels;
    ///<这些数组索引与模型对应即，0：LF，1：LB，2：RB，3：RF
    float wheelVx[4], wheelVy[4], wheelComVel[4];

    float rX = W_FRONT_REAR / 2.f; ///<前后轮距的一半
    float rY = W_LEFT_RIGHT / 2.f; ///<左右轮距的一半
    float carRadius = sqrtf((rX * rX) + (rY * rY));

    if (_speed.v_x == 0 && _speed.v_y == 0 &&
        _speed.w_z ==
                0) { ///<当不进行计算时，所有的轮速均为0 轮子角度保持上一帧的数据不变
        refWheels.S_RF = 0;
        refWheels.S_LF = 0;
        refWheels.S_LB = 0;
        refWheels.S_RB = 0;
        refWheels.M_RF = 0;
        refWheels.M_LF = 0;
        refWheels.M_LB = 0;
        refWheels.M_RB = 0;
    }

    //LF:0
    wheelVx[0] = _speed.v_x - _speed.w_z * rY;
    wheelVy[0] = _speed.v_y + _speed.w_z * rX;
    wheelComVel[0] =
            sqrtf((wheelVx[0] * wheelVx[0]) + (wheelVy[0] * wheelVy[0]));

    expDir.DirAng_LF = atan2(wheelVy[0], wheelVx[0]);

    //LB:1
    wheelVx[1] = _speed.v_x - _speed.w_z * rY;
    wheelVy[1] = _speed.v_y - _speed.w_z * rX;
    wheelComVel[1] =
            sqrtf((wheelVx[1] * wheelVx[1]) + (wheelVy[1] * wheelVy[1]));

    expDir.DirAng_LB = atan2(wheelVy[1], wheelVx[1]);

    //RB:2
    wheelVx[2] = _speed.v_x + _speed.w_z * rY;
    wheelVy[2] = _speed.v_y - _speed.w_z * rX;
    wheelComVel[2] =
            sqrtf((wheelVx[2] * wheelVx[2]) + (wheelVy[2] * wheelVy[2]));

    expDir.DirAng_RB = atan2(wheelVy[2], wheelVx[2]);

    //RF:3
    wheelVx[3] = _speed.v_x + _speed.w_z * rY;
    wheelVy[3] = _speed.v_y + _speed.w_z * rX;
    wheelComVel[3] =
            sqrtf((wheelVx[3] * wheelVx[3]) + (wheelVy[3] * wheelVy[3]));

    expDir.DirAng_RF = atan2(wheelVy[3], wheelVx[3]); //反正切返回(-pi,pi)

    for (uint8_t i = 0; i < 4; i++) {
        if (expDir._[i] < 0) {
            expDir._[i] += 2 * F_PI; //[0,2pi]
        }
    }

    refWheels.M_LF = wheelComVel[0] / W_CIRCUMFERENCE * 60;
    refWheels.M_LB = wheelComVel[1] / W_CIRCUMFERENCE * 60;
    refWheels.M_RB = wheelComVel[2] / W_CIRCUMFERENCE * 60;
    refWheels.M_RF = wheelComVel[3] / W_CIRCUMFERENCE * 60;

    return refWheels;
}

/**
 * @brief 控制电机
 * @note 模型中逆时针角度增大，实际6020id端在上,顺时针编码值增大。
 * @param _refSpeed 
 */
void Steer::ctrl(const Speed_u &_refSpeed)
{
    WheelsSpeed_u refWSpeed = reverse(_refSpeed);

    for (uint8_t i = 0; i < 4; i++) {
        //先决定正反转,要6020转角最小，最好一个对角线上的3508速度直接反向。
        if (expDir._[i] >= F_PI) {
            refWSpeed._[i + 4] = -refWSpeed._[i + 4];
            expDir._[i] -= F_PI; //[0,pi]
            //TODO:此方法限制6020在180°内，但是考虑30°到-30°用上面的方法有明显缺陷，理当求取劣弧。
        }
    }
    for (uint8_t i = 0; i < 4; i++) {
        //6020
        motor_[i]->cmd(MotorCmdType_e::SET_POS, expDir._[i]); //估计会寄
        //3508
        motor_[i + 4]->cmd(MotorCmdType_e::SET_VEL, refWSpeed._[i]);
    }

    if constexpr (USE_POWERCTRL) {
        powerCtrl_->powerCtrl(refWSpeed._);
    }
}