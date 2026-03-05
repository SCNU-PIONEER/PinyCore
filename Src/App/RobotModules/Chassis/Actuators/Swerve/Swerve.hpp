#pragma once
#include "Chassis.hpp"

#include "IMotor.hpp"
#include "INS.hpp"
#include "MsgImpl.hpp"
#include "Pid.hpp"
#include "PidBasic.hpp"
#include "Referee.hpp"
#include "RefereeProt.hpp"
#include "Soc.hpp"
#include "Bsp_can.hpp"
#include "dsp/fast_math_functions.h"
#include "TopicRouter.hpp"

#include "LPF.hpp"
#include "IIR.hpp"

#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "PacketImpl.hpp"
#include "main.h"

namespace CHASSIS {

//以下两个模板专为舵向电机使用(控制较为复杂)
template <typename T> struct Swerve_s {
    T RF;
    T LF;
    T LB;
    T RB;
};

template <typename T> union Swerve_u {
    Swerve_s<T> Chas_s;
    T _[4];
};

union Speed_u {
    struct {
        float vx; // m/s
        float vy; // m/s
        float wz; // rad/s
    } Speed;
    float _[3];
};
union WheelsSpeed_u {
    struct {
        float M_RF; // motor of the right front
        float M_LF; // motor of the left front
        float M_LB; // motor of the left back
        float M_RB; // motor of the right back
    } Wspeed;
    float _[4]; // rad/s
};
//RF 1 ,LF 2 ,LB 3 ,RB 4
union Motors_u {
    struct {
        PINYMOTOR::IMotor *M_RF;
        PINYMOTOR::IMotor *M_LF;
        PINYMOTOR::IMotor *M_LB;
        PINYMOTOR::IMotor *M_RB;

        PINYMOTOR::IMotor *S_RF;
        PINYMOTOR::IMotor *S_LF;
        PINYMOTOR::IMotor *S_LB;
        PINYMOTOR::IMotor *S_RB;
    } Motor;
    PINYMOTOR::IMotor *_[8];
};


class ChassisCtrlRxPacket : public COMM::RxPacket<COMM::ChassisCtrlPacketType, 1> {
    friend class Swerve;
    // TODO: slightly cumbersome like this or CRTP
    ChassisCtrlRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class RefereeCtrlTxPacket : public COMM::TxPacket<COMM::RefereePacketType> {
public:
    RefereeCtrlTxPacket(float _txFreq) : COMM::TxPacket<COMM::RefereePacketType>(_txFreq) {}
    void transmit(uint8_t *_buf, uint16_t _len) final { Can::instance().transmitData(&HCAN3, uid(), _buf, _len); }
};

class SpeedTxPacket : public COMM::TxPacket<COMM::SpeedMsgPacket> {
public:
    SpeedTxPacket(float _txFreq) : COMM::TxPacket<COMM::SpeedMsgPacket>(_txFreq) {}
    void transmit(uint8_t *_buf, uint16_t _len) final { Can::instance().transmitData(&HCAN3, uid(), _buf, _len); }
};

//喵板
class Swerve : public Chassis {
public:
    static constexpr float W_FRONT_REAR = 0.36f; //前后轮距/m
    static constexpr float W_LEFT_RIGHT = 0.36f; //左右轮距
    static constexpr float W_DIAMETER = 0.116f;  //轮子直径
    static constexpr float W_RADIUS = (W_DIAMETER / 2.0f);
    static constexpr float W_CIRCUMFERENCE = (M_PI * W_DIAMETER);
    static constexpr float CAR_RADIUS = 0.25456f; //车体旋转半径

    static constexpr float MAX_VX_SPEED = 2.f;
    static constexpr float MAX_VY_SPEED = 2.f;
    static constexpr float MAX_WZ_SPEED = 10.f;

    Swerve();

    void stop();
    void enter();
    void update(void *_param);
    void ctrl(const Speed_u &_refSpeed);


    void loadDeltaYaw();
    bool isGimbalReset() const;

    const Speed_u &getSpeed() const { return curSpeed_; }

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

    void loadRefereeData(void *_param);

private:
    Motors_u motors_;
    WheelsSpeed_u wSpeed_;
    Speed_u curSpeed_;

    RefereeMsg_s refereeMsg_;

    WheelsSpeed_u refWSpeed_; //期望轮速rad/s

    Swerve_u<float> curDir; //舵轮当前方向角
    Swerve_u<float> expDir; //舵轮期望方向角

    ChassisCtrlRxPacket chassisCtrlRxPacket_;
    RefereeCtrlTxPacket refereeTxPacket_{ 100.f }; //100Hz发送一次
    SpeedTxPacket speedTxPacket{ 100.f };

    REFEREE::SentryCmd_s sentryCmd; //哨兵指令

    Subscriber<INS_SYS::INSData_s> *insSub_;
    INS_SYS::INSData_s insData_ = {};

    float cosYaw_ = 0.f;
    float sinYaw_ = 0.f;
    float del[4], vel[4], sTorq[4], wTorq[4], wCurr[4];

    //舵轮零位角度，用于回正方向,rad
    Swerve_u<float> SwerveZero = { {
            .RF = 5.0083f,
            .LF = 4.95766449f,
            .LB = 6.122f,
            .RB = 1.35620451f,
    } };

    canHandle *wheelCan[4] = { &HCAN2, &HCAN1, &HCAN1, &HCAN2 };
    canHandle *swerveCan[4] = { &HCAN2, &HCAN1, &HCAN1, &HCAN2 };

    //舵轮位置环
    Swerve_u<PositionalPid *> swervePosPid_ = {};
    //舵轮速度环
    Swerve_u<PositionalPid *> swerveVelPid_ = {};
    //舵轮输入滤波器
    FILTER::IIR3 swerveFilterInput_[4];
    //舵轮输出滤波器
    Swerve_u<FILTER::LPF *> swerveFilterOutput_ = {};
    //轮速
    Swerve_u<PositionalPid *> wheelVelPid_ = {};
};

} // namespace CHASSIS
