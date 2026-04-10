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
#include "Swerve.hpp"
#include "dsp/fast_math_functions.h"
#include "TopicRouter.hpp"
#include "LPF.hpp"
#include "IIR.hpp"
#include "main.h"
#include "SwerveController.hpp"
#include "ChassisPacket.hpp"
#include <vector>

namespace CHASSIS {

//以下模板专为舵向电机使用(控制较为复杂)
template <typename T> union Swerve_u {
    struct {
        T RF;
        T LF;
        T LB;
        T RB;
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
        float M_RF; // motor of the right front
        float M_LF; // motor of the left front
        float M_LB; // motor of the left back
        float M_RB; // motor of the right back
    };
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
    };
    PINYMOTOR::IMotor *_[8];
};

template <typename T> struct RobotInteractionData_s {
    uint16_t dataCmdId;  //需为开放的子内容 ID
    uint16_t senderId;   //需与自身 ID 匹配,机器人ID
    uint16_t receiverId; //裁判系统服务器（
    T userdata;          //用户数据
}; //向裁判系统发送的数据 0x0301

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
    bool isGimbalReset;
    bool useSuperCap;

    const Speed_u &getSpeed() const { return curSpeed_; }

    float deltaYaw = 0.f;

protected:
    RefereeMsg_s refereeMsg_;
    REFEREE::RefereeProt_s refereeData = referee->receiver.getRefereeData();
    REFEREE::SentryInfo_s sentryInfo_ = refereeData.sentryInfo; //哨兵信息

    RobotInteractionData_s<REFEREE::SentryCmd_s> sentryCmdData_; //向裁判系统发送的数据 0x0301

    ChassisCtrlRxPacket chassisCtrlRxPacket_;
    RefereeCtrlTxPacket refereeTxPacket_{ 100.f }; //100Hz发送一次
    BulletSpdTxPacket BulletSpdTxPacket_{ 100.f };
    DecesionTxPacket1 decesionPacket1_{ 100.f };
    SentryInfoTxPacket sentryInfoTxPacket_{ 100.f };
    SentryCmdRxPacket sentryCmdRxPacket_; //接收哨兵指令

    void loadRefereeData(void *_param);
    void sendDataToReferee();
    void sendDecesionData();

    float chassisCurPower_;
    float chassisPowerBuffer_;

    float cosYaw_ = 0.f;
    float sinYaw_ = 0.f;
    float sDel[4], sExpVel[4], sTorq[4], wTorq[4], wCurr[4];

    Speed_u forward(const WheelsSpeed_u &_wSpeed);
    WheelsSpeed_u reverse(const Speed_u &_speed);

private:
    Motors_u motors_;

    Subscriber<INS_SYS::INSData_s> *insSub_;
    INS_SYS::INSData_s insData_ = {};

    SwerveController *powerCtrl_;
    CAP cap_{ &HCAN2 };

    WheelsSpeed_u wSpeed_;
    Speed_u curSpeed_;
    WheelsSpeed_u refWSpeed_; //期望轮速rad/s

    Swerve_u<float> curDir;   //舵轮当前方向角
    Swerve_u<float> expDir;   //舵轮期望方向角
    Swerve_u<float> sCurVel;  //舵轮当前速度
    Swerve_u<float> sCurTorq; //舵轮当前扭矩

    Swerve_u<float> SwerveZero = {
        { .RF = 5.0083f, .LF = 4.95766449f, .LB = 3.41889358f, .RB = 1.35620451f }
    }; //舵轮零位角度，用于回正方向,rad

    canHandle *wheelCan[4] = { &HCAN2, &HCAN1, &HCAN1, &HCAN2 };
    canHandle *swerveCan[4] = { &HCAN2, &HCAN1, &HCAN1, &HCAN2 };

    Swerve_u<PositionalPid *> swervePosPid_ = {};         //舵轮位置环
    Swerve_u<PositionalPid *> swerveVelPid_ = {};         //舵轮速度环
    Swerve_u<FILTER::LPF *> swerveFilterInput_ = {};      //舵轮输入滤波器
    Swerve_u<FILTER::LPFIIR3 *> swerveFilterOutput_ = {}; //舵轮输出滤波器
    Swerve_u<PositionalPid *> wheelVelPid_ = {};          //轮速
};

} // namespace CHASSIS
