#pragma once

#include "Gimbal.hpp"

#include "IMotor.hpp"

#include "LPF.hpp"
#include "MotorCommonMacros.hpp"
#include "PidBasic.hpp"

namespace GIMBAL {

//指针模板
template <typename T> union DualStagePtr_u {
    struct {
        T *Pitch;
        T *FirstYaw;  // first stage yaw motor, 大yaw
        T *SecondYaw; // second stage yaw motor, 小yaw
    };
    T *_[3];
};
//非指针模板
template <typename T> union DualStage_u {
    struct {
        T Pitch;
        T FirstYaw;  //大yaw
        T SecondYaw; //小yaw
    };
    T _[3];
};

//转换为-pi~pi
static inline float rangeConversion(float _ang)
{
    return _ang > PI ? _ang - (2 * PI) : (_ang < -PI ? _ang + (2 * PI) : _ang);
};
/**
 * @brief 双级yaw轴,大小yaw.
 * @note 当前版本imu装在小yaw上,但是控制的是大yaw
 */
class DualStage : public Gimbal {
public:
    static constexpr float PITCH_LAUNCH_ANG = 2.055018f;
    static constexpr float FIRST_YAW_LAUNCH_ANG = 6.22f;
    static constexpr float SECOND_YAW_LAUNCH_ANG = 4.1722f;

    static constexpr float PITCH_MOTOR_FREQ = 1000.f;
    static constexpr float FIRST_YAW_MOTOR_FREQ = 1000.f;
    static constexpr float SECOND_YAW_MOTOR_FREQ = 1000.f;

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool PITCH_REVERSE = false;
    static constexpr bool FIRST_YAW_REVERSE = false;
    static constexpr bool SECOND_YAW_REVERSE = false;

    DualStage();


    DualStage_u<bool> isGimbalResverse;

    DualStage_u<float> curImuAng_ = {};   //-pi~pi
    DualStage_u<float> expImuAng_ = {};   //-pi~pi
    DualStage_u<float> curMotorAng_ = {}; //0~2pi
    DualStage_u<float> expMotorAng_ = {}; //0~2pi

    void stop();
    void enter();
    void update(void *_param);

    void ctrl(DualStage_u<float> _pos, DualStagePtr_u<PositionalPid> _angPID,
              DualStagePtr_u<PositionalPid> _velPID);
    void ctrl(DualStage_u<float> _pos, DualStage_u<float> _vel,
              DualStagePtr_u<PositionalPid> _angPID,
              DualStagePtr_u<PositionalPid> _velPID);

    DualStagePtr_u<PINYMOTOR::IMotor> *motors() { return &motors_; }
    DualStagePtr_u<FILTER::LPF> *gimbalFilter() { return &gimbalFilterOutput_; }

protected:
private:
    /** 
    * @note 上电后云台系与底盘系会有差角,(imu上电即为0,而云台当前角度不一定为0)
    *       imu系需要补偿此差角得到当前云台数据,叫gimbalYawAng可能会更好理解;
    */
    void compensateImuAngle();
    void updateBaseYaw();

    DualStagePtr_u<FILTER::LPF> gimbalFilterOutput_; //输出滤波
    DualStagePtr_u<PINYMOTOR::IMotor> motors_;       //电机
};

} // namespace GIMBAL
