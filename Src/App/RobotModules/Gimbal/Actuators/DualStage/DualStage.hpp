#pragma once

#include "Gimbal.hpp"

#include "IMotor.hpp"

#include "LPF.hpp"
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
        T FirstYaw;
        T SecondYaw;
    };
    T _[3];
};

struct Ang_s {
    float roll, yaw, pitch;
};

union GimbalCmdPara_u {
    struct {
        float pos;
        float vel;
        float torq;
    };
    float _[3];
};

/**
 * @brief 双级yaw轴,大小yaw.
 * @note 当前版本imu装在小yaw上,但是控制的是大yaw
 */
class DualStage : public Gimbal {
public:
    static constexpr float PITCH_LAUNCH_ANG = 0.f;
    static constexpr float FIRST_YAW_LAUNCH_ANG = 0.f;
    static constexpr float SECOND_YAW_LAUNCH_ANG = 0.f;

    static constexpr float PITCH_MOTOR_FREQ = 500.f;
    static constexpr float FIRST_YAW_MOTOR_FREQ = 500.f;
    static constexpr float SECOND_YAW_MOTOR_FREQ = 500.f;

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool PITCH_REVERSE = false;
    static constexpr bool FIRST_YAW_REVERSE = false;
    static constexpr bool SECOND_YAW_REVERSE = false;

    DualStage();

    DualStagePtr_u<FILTER::LPF> gimbalFilterOutput_; //输出滤波

    void stop();
    void enter();
    void update(void *_param);

    void ctrl(PINYMOTOR::IMotor *_motor, FILTER::LPF *_filter, PID *_angPID,
              PID *_velPID, float _pos);
    void ctrl(PINYMOTOR::IMotor *_motor, FILTER::LPF *_filter, PID *_angPID,
              PID *_velPID, float _pos, float _vel);

    DualStagePtr_u<PINYMOTOR::IMotor> *motors() { return &motors_; }
    DualStagePtr_u<FILTER::LPF> *gimbalFilter() { return &gimbalFilterOutput_; }


protected:
    INS_SYS::INSData_s secondYawInsData_ = insData_; // 云台imu在小yaw上
    Ang_s firstYawAng_ = {};

private:
    void updateCtrlYaw();
    void updateBaseYaw();

    DualStagePtr_u<PINYMOTOR::IMotor> motors_; //电机
};

} // namespace GIMBAL
