#pragma once

#include "Gimbal.hpp"

#include "sdkconfig.h"

#include "IMotor.hpp"

#include "LPF.hpp"

namespace GIMBAL {

union Motors_u {
    struct {
        PINYMOTOR::IMotor *pitch;
        PINYMOTOR::IMotor *yaw;
    };
    PINYMOTOR::IMotor *_[2];
};

union States_u {
    struct {
        float pos;
        float vel;
        float torq;
    };
    float _[2];
};

class Standard : public Gimbal {
public:
    static constexpr float PITCH_LAUNCH_ANG = 2.08033204f;
    static constexpr float YAW_LAUNCH_ANG = 1.28333163f;

    static constexpr float PITCH_MOTOR_FREQ = 500.f;
    static constexpr float YAW_MOTOR_FREQ = 500.f;

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool PITCH_REVERSE = false;
    static constexpr bool YAW_REVERSE = false;

    Standard();

    PINYMOTOR::IMotor *pitch() const;
    PINYMOTOR::IMotor *yaw() const;

    void ctrlPitchSmooth(PID *_angPID, PID *_velPID, float _pos, float _step);
    void ctrlPitch(PID *_angPID, PID *_velPID, float _pos);

    void ctrlYawSmooth(PID *_angPID, PID *_velPID, float _pos, float _step);
    void ctrlYaw(PID *_angPID, PID *_velPID, float _pos);

    void stop();
    void enter();
    void update(void *_param);

    void loadResetState(bool _state);

private:
    Motors_u motors_;

    // Master send deltaYaw to chassis
    ChassisCtrlMsg_s chassisCtrlMsg_{};
    Publisher<ChassisCtrlMsg_s> *chassisCtrlPub_;

    FILTER::LPF outFilrPitch_{ PITCH_MOTOR_FREQ, 1000.f };
    FILTER::LPF outFilrYaw_{ YAW_MOTOR_FREQ, 1000.f };
};

} // namespace GIMBAL
