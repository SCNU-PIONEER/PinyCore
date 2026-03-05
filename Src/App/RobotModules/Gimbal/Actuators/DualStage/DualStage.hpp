#pragma once

#include "Gimbal.hpp"

#include "IMotor.hpp"
#include "RefereeProt.hpp"
#include "Vision.hpp"
#include "Nav.hpp"
#include "LPF.hpp"
#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"
#include "PacketImpl.hpp"
#include "TxPacket.hpp"
#include "RxPacket.hpp"

namespace GIMBAL {
template <typename T> struct DualStage_s {
    T Pitch;
    T FirstYaw;  //大yaw
    T SecondYaw; //小yaw
};
template <typename T> union DualStage_u {
    DualStage_s<T> Gim_s;
    T _[3];
};

struct DualStageDebug_s {
    bool visionControl;
    bool visionFire;
    float visionTargetYaw;
};

//转换为-pi~pi
static inline float rangeConversion(float _ang)
{
    return _ang > PI ? _ang - (2 * PI) : (_ang < -PI ? _ang + (2 * PI) : _ang);
};

//红1机器人ID 为 1，蓝1 机器人 ID 为 101
inline bool isColor(float _robotId)
{
    if (_robotId < 100) {
        return 1; // 红色
    } else {
        return 0; // 蓝色
    }
}

class ChassisCtrlTxPacket : public COMM::TxPacket<COMM::ChassisCtrlPacketType> {
    friend class DualStage;
    // ChassisCtrlTxPacket() : TxPacket(200.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class ShooterCtrlTxPacket : public COMM::TxPacket<COMM::ArmorBoosterPacketType> {
    friend class DualStage;
    // ShooterCtrlTxPacket() : TxPacket(200.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class RefereeRxPacket : public COMM::RxPacket<COMM::RefereePacketType, 1> {
    friend class DualStage;
    RefereeRxPacket() { registerCallback(); }
    void registerCallback() final;
};

#define _no_init __attribute__((section(".noinit_ram")))

/**
 * @brief 双级yaw轴,大小yaw.
 * @note 当前版本imu装在小yaw上,但是控制的是大yaw
 */
class DualStage : public Gimbal {
public:
    static constexpr float PITCH_LAUNCH_ANG = 1.22764301f;
    static constexpr float FIRST_YAW_LAUNCH_ANG = 2.826614875f;
    static constexpr float SECOND_YAW_LAUNCH_ANG = 4.30871105f;

    static constexpr float PITCH_MOTOR_FREQ = 1000.f;
    static constexpr float FIRST_YAW_MOTOR_FREQ = 1000.f;
    static constexpr float SECOND_YAW_MOTOR_FREQ = 1000.f;

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool PITCH_REVERSE = true;
    static constexpr bool FIRST_YAW_REVERSE = false;
    static constexpr bool SECOND_YAW_REVERSE = true;

    static constexpr float ANGLE_JUMP_VALUE = 0.033169213f; // rad,现状为每圈只跳一次

    static constexpr float YAW_INERTIA = 0.0195103455f; //牢全参数
    static constexpr float YAW_FRICTION = 0.00570616219f;
    static constexpr float YAW_COULOMB = 0.0173312854f;

    DualStage();

    DualStageDebug_s debug{};

    bool isVisionAngOver_ = false; //视觉角度是否超出小yaw控制范围
    DualStage_u<bool> isGimbalResverse;
    DualStage_u<float> curImuAng_ = {};   //-pi~pi
    DualStage_u<float> expImuAng_ = {};   //-pi~pi
    DualStage_u<float> curMotorAng_ = {}; //0~2pi
    DualStage_u<float> expMotorAng_ = {}; //0~2pi

    void stop();
    void enter();
    void update(void *_param);
    void loadResetState(bool _state);

    void ctrl(DualStage_u<float> _pos, DualStage_u<PositionalPid *> _angPID, DualStage_u<PositionalPid *> _velPID);
    void ctrlSmooth(DualStage_u<float> _pos, DualStage_u<float> _step, DualStage_u<PositionalPid *> _angPID,
                    DualStage_u<PositionalPid *> _velPID);
    void ctrlSecondYawAuto(PID *_angPID, PID *_velPID, PID *_velFeedbackPID, float _pos);

    DualStage_u<PINYMOTOR::IMotor *> *motors() { return &motors_; }
    DualStage_u<FILTER::LPF *> *gimbalFilter() { return &gimbalFilterOutput_; }

protected:
    /**
        * @brief 双轴云台就是大小yaw，控制大yaw
        * @note 上电后云台系与底盘系会有差角,(imu上电即为0,而云台当前角度不一定为0)
        *       imu系需要补偿此差角得到当前云台数据,叫gimbalYawAng可能会更好理解;
        */
    float ctrlYawAng_ = 0.f;        //主控yaw的角度
    float baseYawAng_ = 0.f;        // 底盘跟随，单板要用，双板直接读取imu
    float constDeltaYawArc_ = 0.f;  // 上电后就有的底盘和云台系的差角
    bool isConfirmYawZero_ = false; // 确认是否两系转换完成

    void compensateImuAngle();
    void updateBaseYaw();

    void loadChassisMsg(void *_param);
    void loadShooterMsg(void *_param);
    void sendVisionData();
    void sendNavData();

    void resetZeroPoint(); //大yaw跳齿

    float calcYawTorq(float _vel, float _acc);

private:
    ChassisMsg_s cmsg{};
    NavMsg_s nmsg{};

    ChassisCtrlTxPacket chassisCtrlTxPacket_;
    ShooterCtrlTxPacket shooterCtrlTxPacket_;
    RefereeRxPacket refereeRxPacket_; //裁判系统

    DualStage_u<FILTER::LPF *> gimbalFilterOutput_; //输出滤波
    DualStage_u<PINYMOTOR::IMotor *> motors_;       //电机
};

} // namespace GIMBAL
