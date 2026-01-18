#pragma once
#include "Chassis.hpp"

#include "IMotor.hpp"
#include "INS.hpp"
#include "Pid.hpp"
#include "PidBasic.hpp"
#include "Soc.hpp"
#include "dsp/fast_math_functions.h"
#include "TopicRouter.hpp"

#include "LPF.hpp"

#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "PacketImpl.hpp"
#include "main.h"

namespace CHASSIS {

//以下两个模板专为舵向电机使用(控制较为复杂)
template <typename T> union SwervePtr_u {
    struct {
        T *S_LF;
        T *S_RF;
        T *S_RB;
        T *S_LB;
    };
    T *_[4];
};
template <typename T> union Swerve_u {
    struct {
        T S_LF;
        T S_RF;
        T S_RB;
        T S_LB;
    };
    T _[4];
};

union Speed_u {
    struct {
        float vx; // m/s
        float vy; // m/s
        float wz; // rad/s
    };
    float _[3];
};
union WheelsSpeed_u {
    struct {
        float M_LF; // motor of the left front
        float M_RF; // motor of the right front
        float M_RB; // motor of the right back
        float M_LB; // motor of the left back
    };
    float _[4]; // rpm
};

union Motors_u {
    struct {
        PINYMOTOR::IMotor *M_LF;
        PINYMOTOR::IMotor *M_RF;
        PINYMOTOR::IMotor *M_RB;
        PINYMOTOR::IMotor *M_LB;

        PINYMOTOR::IMotor *S_LF;
        PINYMOTOR::IMotor *S_RF;
        PINYMOTOR::IMotor *S_RB;
        PINYMOTOR::IMotor *S_LB;
    };
    PINYMOTOR::IMotor *_[8];
};

class ChassisCtrlRxPacket : public COMM::RxPacket<COMM::ChassisCtrlPacketType, 1> {
    friend class Swerve;
    // TODO: slightly cumbersome like this or CRTP
    ChassisCtrlRxPacket() { registerCallback(); }
    void registerCallback() final;
};

//喵板
class Swerve : public Chassis {
public:
    static constexpr float W_FRONT_REAR = 0.298f; //前后轮距/m
    static constexpr float W_LEFT_RIGHT = 0.298f; //左右轮距
    static constexpr float W_DIAMETER = 0.098f;   //轮子直径
    static constexpr float W_RADIUS = (W_DIAMETER / 2.0f);
    static constexpr float W_CIRCUMFERENCE = (M_PI * W_DIAMETER);
    static constexpr float CAR_RADIUS = 0.210f; //车体旋转半径

    Swerve();

    void stop();
    void enter();
    void update(void *_param);
    void ctrl(const Speed_u &_refSpeed);

    void loadDeltaYaw();
    bool isGimbalReset() const;

    float deltaYaw = 0.f;

protected:
    /*
     * @brief: calculate the speed of chassis
     * @param: _wSpeed: speed of each wheel
     * @return: speed of the chassis in m/s and rad/s
    */
    Speed_u forward(const WheelsSpeed_u &_wSpeed);

    /*
     * @brief: calculate the speed of each wheel
     * @param: _speed: speed of the chassis in m/s and rad/s
     * @return: speed of each wheel
    */
    WheelsSpeed_u reverse(const Speed_u &_speed);

private:
    Motors_u motors_;
    WheelsSpeed_u wSpeed_;
    Speed_u curSpeed_;

    Swerve_u<float> curDir; //舵轮当前方向角
    Swerve_u<float> expDir; //舵轮期望方向角

    ChassisCtrlRxPacket chassisCtrlRxPacket_;

    Subscriber<INS_SYS::INSData_s> *insSub_;
    INS_SYS::INSData_s insData_ = {};

    float cosYaw_ = 0.f;
    float sinYaw_ = 0.f;

    //舵轮零位角度，用于回正方向,rad
    float SwerveZero[4] = {
        3.6206367f,
        0.556135f,
        4.248f,
        3.03151608f,
    };

    Swerve_u<canHandle> wheelCan = {
        .S_LF = HCAN1,
        .S_RF = HCAN2,
        .S_RB = HCAN2,
        .S_LB = HCAN1,
    };

    Swerve_u<canHandle> swerveCan = {
        .S_LF = HCAN1,
        .S_RF = HCAN2,
        .S_RB = HCAN2,
        .S_LB = HCAN1,
    };

    //舵轮位置环
    SwervePtr_u<PositionalPid> swervePosPid_ = {
        new PositionalPid(15.f, 0.001f, 0.f, 1.f / 1000.f, 0.f, 35.f, 0.f),
        new PositionalPid(15.f, 0.001f, 0.f, 1.f / 1000.f, 0.f, 35.f, 0.f),
        new PositionalPid(15.f, 0.001f, 0.f, 1.f / 1000.f, 0.f, 35.f, 0.f),
        new PositionalPid(15.f, 0.001f, 0.f, 1.f / 1000.f, 0.f, 35.f, 0.f),
    };
    //舵轮速度环
    SwervePtr_u<PositionalPid> swerveVelPid_ = {
        new PositionalPid(0.01f, 0.001f, 0.f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
        new PositionalPid(0.01f, 0.001f, 0.f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
        new PositionalPid(0.01f, 0.001f, 0.f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
        new PositionalPid(0.01f, 0.001f, 0.f, 1.f / 1000.f, 0.3f, 1.2f, 0.f),
    };
    //舵轮输出滤波器
    SwervePtr_u<FILTER::LPF> swerveFilterOutput_ = {
        new FILTER::LPF(1000.f, 4.f),
        new FILTER::LPF(1000.f, 4.f),
        new FILTER::LPF(1000.f, 4.f),
        new FILTER::LPF(1000.f, 4.f),
    };
};

} // namespace CHASSIS
