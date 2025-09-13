#pragma once

#include "Gimbal.hpp"

#include "sdkconfig.h"

#include <functional>

#include "IMotor.hpp"

#include "LPF.hpp"

#include "TxPacket.hpp"
#include "PacketImpl.hpp"

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

class ChassisCtrlTxPacket : public COMM::TxPacket<COMM::SamplePacketType> {
    void send(uint8_t *_buf, uint16_t _len) final;
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

    float ctrlYawAng() const { return ctrlYawAng_; }

    float baseYawAng() const { return baseYawAng_; }

    void loadResetState(bool _state);

    FILTER::LPF outFilrPitch{ PITCH_MOTOR_FREQ, 4.f };
    FILTER::LPF outFilrYaw{ YAW_MOTOR_FREQ, 3.f };

private:
    void updateCtrlYaw();
    void updateBaseYaw();

    Motors_u motors_;

#if !APP_USE_COMM
    // Master send deltaYaw to chassis
    GimbalYawMsg_s gimbalYawMsg_{};
    Publisher<GimbalYawMsg_s> *gimbalYawPub_;
#endif
#ifdef APP_GIMBAL_MASTER
    ChassisCtrlTxPacket chassisCtrlTxPacket_;
#endif

    float ctrlYawAng_ = 0.f;       // be-ctrled-Joint's ang in earth
    float constDeltaYawArc_ = 0.f; // be-ctrled-Joint to base mapping arc
    bool isConfirmYawZero_ = false;

    float baseYawAng_ = 0.f; // J0's ang in earth
};

} // namespace GIMBAL
