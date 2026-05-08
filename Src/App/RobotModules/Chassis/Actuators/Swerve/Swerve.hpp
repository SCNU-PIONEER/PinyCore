#pragma once
#include "PowerController.hpp"
#include "main.h"
#include "Soc.hpp"
#include "MsgImpl.hpp"
#include "Chassis.hpp"
#include "IMotor.hpp"
#include "INS.hpp"
#include "LPF.hpp"
#include "PidBasic.hpp"
#include "ChassisPacket.hpp"
#include "SuperCap.hpp"
#include "Referee.hpp"
#include "RefereeProt.hpp"

#if APP_USE_POWERCTRL
class SwerveController;
#endif

namespace CHASSIS {

template <typename T> union SwervePower_u {
    struct {
        T W_RF;
        T W_LF;
        T W_LB;
        T W_RB;

        T S_RF;
        T S_LF;
        T S_LB;
        T S_RB;
    };
    T _[8];
};

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

typedef struct {
    float chassisCurPower;
    float chassisPowerBuffer;
    Swerve_u<float> sExpVel, sExpTorq, wExpTorq, wCurr;
} SwerveDebug_s;

template <typename T> struct RobotInteractionData_s {
    uint16_t dataCmdId;  //需为开放的子内容 ID
    uint16_t senderId;   //需与自身 ID 匹配,机器人ID
    uint16_t receiverId; //裁判系统服务器
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

    static constexpr float STEER_MOTOR_FREQ = 500.f;
    static constexpr float WHEEL_MOTOR_FREQ = 500.f;

    Swerve();

    void stop();
    void enter();
    void update(void *_param);
    void ctrl(const Speed_u &_refSpeed);

    float deltaYaw = 0.f;
    void loadDeltaYaw();
    bool isGimbalReset = false;

    const Speed_u &getSpeed() const { return curSpeed_; }

protected:
    RefereeMsg_s refereeMsg_;
    REFEREE::RefereeProt_s refereeData_;
    REFEREE::SentryInfo_s sentryInfo_; //哨兵信息

    RobotInteractionData_s<REFEREE::SentryCmd_s> sentryCmdData_; //向裁判系统发送的数据 0x0301

    ChassisCtrlRxPacket chassisCtrlRxPacket_;
    RefereeCtrlTxPacket refereeTxPacket_{ 100.f };
    BulletSpdTxPacket BulletSpdTxPacket_{ 100.f };
    DecesionTxPacket1 decesionTxPacket1_{ 100.f };
    SentryInfoTxPacket sentryInfoTxPacket_{ 100.f };
    SentryCmdRxPacket sentryCmdRxPacket_; //接收哨兵指令

    float cosYaw_ = 0.f;
    float sinYaw_ = 0.f;

    void loadRefereeData(void *_param);
    void sendDataToReferee();
    void sendDecesionData();

    Speed_u forward(const Swerve_u<float> &_wSpeed);
    Swerve_u<float> reverse(const Speed_u &_speed);

private:
    Subscriber<INS_SYS::INSData_s> *insSub_;
    INS_SYS::INSData_s insData_ = {};

    Swerve_u<PINYMOTOR::IMotor *> steerMotor_;
    Swerve_u<PINYMOTOR::IMotor *> wheelMotor_;

#if APP_USE_POWERCTRL
    SwerveController *powerCtrl_;

#endif
#if EXTENSION_SUPERCAP
    SuperCap cap_{ &HCAN2 };
    float capCmdRatio_;
#endif
    Speed_u curSpeed_ = {};          //当前车体速度
    Swerve_u<float> curWSpeed_ = {}; //实际轮速rad/s
    Swerve_u<float> expWSpeed_ = {}; //期望轮速rad/s

    Swerve_u<float> curDir = {};   //舵轮当前方向角
    Swerve_u<float> expDir = {};   //舵轮期望方向角
    Swerve_u<float> sCurVel = {};  //舵轮当前速度
    Swerve_u<float> sCurTorq = {}; //舵轮当前扭矩

    SwerveDebug_s debug_ = {};

    Swerve_u<float> steerZero = { {
            .RF = 5.0083f,
            .LF = 4.95766449f,
            .LB = 3.41889358f,
            .RB = 5.52453974f,
    } }; //舵轮零位角度，用于回正方向,rad

    Swerve_u<canHandle *> wheelCan = { { .RF = &HCAN2, .LF = &HCAN1, .LB = &HCAN1, .RB = &HCAN2 } };
    Swerve_u<canHandle *> steerCan = { { .RF = &HCAN2, .LF = &HCAN1, .LB = &HCAN1, .RB = &HCAN2 } };

    Swerve_u<FILTER::LPF *> steerFilterInput_ = {}; //舵轮输入滤波器
    Swerve_u<PositionalPid *> steerPosPid_ = {};    //舵轮位置环
    Swerve_u<PositionalPid *> steerVelPid_ = {};    //舵轮速度环
    Swerve_u<PositionalPid *> wheelVelPid_ = {};    //轮速pid
    Swerve_u<PositionalPid *> sBumpVelPid_ = {};    //舵轮减速带速度环
    Swerve_u<PositionalPid *> wBumpVelPid_ = {};    //减速带专用轮速pid
};

} // namespace CHASSIS
