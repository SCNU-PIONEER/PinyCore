#pragma once

#include "Gimbal.hpp"

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
    static constexpr float YAW_LAUNCH_ANG = 4.42377377f;

    static constexpr float PITCH_MOTOR_FREQ = 500.f;
    static constexpr float YAW_MOTOR_FREQ = 500.f;

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool PITCH_REVERSE = false;
    static constexpr bool YAW_REVERSE = false;

    Standard();

    PINYMOTOR::IMotor *pitch() const;
    PINYMOTOR::IMotor *yaw() const;

    void ctrlPitch(PID *_angPID, PID *_velPID, float _pos);
    void ctrlPitch(PID *_angPID, PID *_velPID, float _pos, float _vel);

    void ctrlYaw(PID *_angPID, PID *_velPID, float _pos);
    void ctrlYaw(PID *_angPID, PID *_velPID, float _pos, float _vel);

    void stop();
    void enter();
    void update(void *_param);

    FILTER::LPF outFilrPitch{ PITCH_MOTOR_FREQ, 4.f };
    FILTER::LPF outFilrYaw{ YAW_MOTOR_FREQ, 3.f };

private:
    void updateCtrlYawAng();
    void updateBaseYaw();

    Motors_u motors_;
};

} // namespace GIMBAL
