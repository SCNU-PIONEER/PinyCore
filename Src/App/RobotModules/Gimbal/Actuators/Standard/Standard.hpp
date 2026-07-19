#pragma once

#include "Gimbal.hpp"

#include "sdkconfig.h"

#include "IMotor.hpp"

#include "LPF.hpp"

#include "TxPacket.hpp"
#include "RxPacket.hpp"
#include "PacketImpl.hpp"

namespace GIMBAL {

class ChassisCtrlTxPacket : public COMM::TxPacket<COMM::ChassisCtrlPacketType> {
    friend class Standard;
    // ChassisCtrlTxPacket() : TxPacket(200.f) {}
    void transmit(Data &_data) final;
};

class ShooterCtrlTxPacket : public COMM::TxPacket<COMM::ArmorBoosterPacketType> {
    friend class Standard;
    // ShooterCtrlTxPacket() : TxPacket(200.f) {}
    void transmit(Data &_data) final;
};

class Standard : public Gimbal {
    union Motors_u {
        struct {
            PINYMOTOR::IMotor *pitch;
            PINYMOTOR::IMotor *yaw;
        };
        PINYMOTOR::IMotor *_[2];
    };

public:
    static constexpr float PITCH_LAUNCH_ANG = 6.25825691f;
    static constexpr float YAW_LAUNCH_ANG = 4.74211359f;

    static constexpr float PITCH_MOTOR_FREQ = 800.f;
    static constexpr float YAW_MOTOR_FREQ = 800.f;

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool PITCH_REVERSE = false;
    static constexpr bool YAW_REVERSE = true;

    Standard();

    // Vision vision;

    PINYMOTOR::IMotor *pitch() const;
    PINYMOTOR::IMotor *yaw() const;

    void ctrlPitchSmooth(PID *_angPID, PID *_velPID, float _pos, float _step);
    void ctrlPitchAuto(PID *_angPID, PID *_velPID, float _pos);
    void ctrlPitch(PID *_angPID, PID *_velPID, float _pos);

    void ctrlYawSmooth(PID *_angPID, PID *_velPID, float _pos, float _step);
    void ctrlYawAuto(PID *_angPID, PID *_velPID, float _pos);
    void ctrlYaw(PID *_angPID, PID *_velPID, float _pos);

    void stop();
    void enter();
    void update(void *_param);

    void loadResetState(bool _state);

private:
    Motors_u motors_;
    ChassisMsg_s cmsg{};
    NavMsg_s nmsg{};

    ChassisCtrlTxPacket chassisCtrlTxPacket_;
    ShooterCtrlTxPacket shooterCtrlTxPacket_;

    FILTER::LPF outFilrPitch_{ PITCH_MOTOR_FREQ, 1000.f };
    FILTER::LPF outFilrYaw_{ YAW_MOTOR_FREQ, 1000.f };

    float calcPitchTorq(float _ang, float _vel, float _acc);
    float calcYawTorq(float _vel, float _acc);
};

} // namespace GIMBAL
