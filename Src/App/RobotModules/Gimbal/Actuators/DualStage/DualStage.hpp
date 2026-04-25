#pragma once

#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "Bsp_can.hpp"
#include "IMotor.hpp"
#include "MsgImpl.hpp"
#include "RefereeProt.hpp"
#include "MiniPC.hpp"
#include "LPF.hpp"
#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include "GimbalPacket.hpp"

namespace GIMBAL {

template <typename T> union DualStage_u {
    struct {
        T Pitch;
        T FirstYaw;  //大yaw
        T SecondYaw; //小yaw
    };
    T _[3];
};

struct DualStageDebug_s {
    bool visionControl;
    bool visionFire;
    bool visionAngOver;
    float visionYaw;
    float visionPitch;
    FSMState_e gimbalState;
};

//转换为-pi~pi
static inline float rangeConversion(float _ang)
{
    // return _ang > PI ? _ang - (2 * PI) : (_ang < -PI ? _ang + (2 * PI) : _ang);
    return PINYMOTOR::rangeMap(_ang, -M_PI, M_PI);
};

//红1机器人ID为1，蓝1机器人ID为101,true红
inline bool isColor(float _robotId)
{
    if (_robotId < 100) {
        return 1; // 己红,对蓝
    } else {
        return 0; // 己蓝,对红
    }
}


// #define _no_init __attribute__((section(".noinit_ram")))

/**
 * @brief 双级yaw轴,大小yaw.
 * @note 当前版本imu装在小yaw上,但是控制的是大yaw
 * @note 尝试大yaw跟小yaw，小yaw跟imu
 */
class DualStage : public Gimbal {
public:
    static constexpr float PITCH_LAUNCH_ANG = 1.24988747f;
    static constexpr float FIRST_YAW_LAUNCH_ANG = 1.09417856f;
    static constexpr float SECOND_YAW_LAUNCH_ANG = 4.63702297f;

    static constexpr float PITCH_MOTOR_FREQ = 1000.f;
    static constexpr float FIRST_YAW_MOTOR_FREQ = 1000.f;
    static constexpr float SECOND_YAW_MOTOR_FREQ = 1000.f;

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool PITCH_REVERSE = true;
    static constexpr bool FIRST_YAW_REVERSE = false;
    static constexpr bool SECOND_YAW_REVERSE = true;

    static constexpr float MAX_BYAW_FOLLOW_SPEED = 10.f;

    static constexpr float PITCH_MAX_ANG = 0.4131106f; // 24°
    static constexpr float PITCH_MIN_ANG = -0.6012f;   // -35°
    static constexpr float SECOND_YAW_MAX_ANG = 0.22f; // 13°
    static constexpr float SECOND_YAW_MIN_ANG = -0.22f;

    static constexpr float YAW_INERTIA = 0.0195103455f; //牢全参数
    static constexpr float YAW_FRICTION = 0.00570616219f;
    static constexpr float YAW_COULOMB = 0.0173312854f;

    DualStage();

    MiniPC miniPC_;
    VisionToEc_s v2eData;
    NavToEc_s n2eData;
    DecesionToEc_s d2eData;
    EcToVision_s &e2vData = miniPC_.setData().e2vData;
    EcToNav_s &e2nData = miniPC_.setData().e2nData;
    EcToDecesion_s &e2dData = miniPC_.setData().e2dData;

    DualStageDebug_s debug = {};
    bool isLaunched_ = false;
    bool isVisionAngOver_ = false; //视觉角度是否超出小yaw控制范围
    DualStage_u<bool> isGimbalResverse;
    DualStage_u<float> curImuAng_ = {};   //-pi~pi
    DualStage_u<float> expImuAng_ = {};   //-pi~pi
    DualStage_u<float> curMotorAng_ = {}; //0~2pi
    DualStage_u<float> expMotorAng_ = {}; //0~2pi

    void stop();
    void enter();
    void update(void *_param);

    void ctrl(DualStage_u<float> _insPos, DualStage_u<PositionalPid *> _angPID, DualStage_u<PositionalPid *> _velPID);
    void ctrlMotor(DualStage_u<float> _motorAng, DualStage_u<PositionalPid *> _angPID,
                   DualStage_u<PositionalPid *> _velPID);
    void ctrlSmooth(DualStage_u<float> _pos, DualStage_u<float> _step, DualStage_u<PositionalPid *> _angPID,
                    DualStage_u<PositionalPid *> _velPID);
    void ctrlSecondYawAuto(PID *_angPID, PID *_velPID, PID *_velFeedbackPID, float _pos);

    DualStage_u<FILTER::LPF *> *gimbalFilter() { return &gimbalFilterOutput_; }

protected:
    void loadChassisMsg(void *_param);
    void loadShooterMsg(void *_param);
    void loadSentryCmd();
    void sendVisionData();
    void sendNavData();
    void sendDecesionData();

    float calcYawTorq(float _vel, float _acc);

private:
    ChassisMsg_s cmsg{};

    ChassisCtrlTxPacket chassisCtrlTxPacket_{ 300.f };
    ShooterCtrlTxPacket shooterCtrlTxPacket_{ 500.f };
    RefereeRxPacket refereeRxPacket_;              //裁判系统
    BulletSpdRxPacket bulletSpdRxPacket_;          //弹速
    DecesionRxPacket1 decesionRxPacket1_;          //决策数据
    SentryInfoRxPacket sentryInfoRxPacket_;        //哨兵信息
    SentryCmdTxPacket sentryCmdTxPacket_{ 100.f }; //哨兵命令

    DualStage_u<FILTER::LPF *> gimbalFilterOutput_; //输出滤波
    DualStage_u<PINYMOTOR::IMotor *> motors_;       //电机
};

} // namespace GIMBAL
