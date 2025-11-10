#pragma once

#include "Gimbal.hpp"
#include "IMotor.hpp"
#include "LPF.hpp"
#include "TxPacket.hpp"
#include "PacketImpl.hpp"

namespace GIMBAL {

class ChassisCtrlTxPacket : public COMM::TxPacket<COMM::ChassisCtrlPacketType> {
    friend class Standard;
    // ChassisCtrlTxPacket() : TxPacket(500.f) {}
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class Standard : public Gimbal {
public:
    static constexpr float YAW_LAUNCH_ANG = 4.74211359f;

    static constexpr float YAW_MOTOR_FREQ = 500.f;

    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool YAW_REVERSE = true;

    Standard();

    PINYMOTOR::IMotor *yaw() const;

    void ctrlYawSmooth(PID *_angPID, PID *_velPID, float _pos, float _step);
    void ctrlYaw(PID *_angPID, PID *_velPID, float _pos);

    void stop();
    void enter();
    void update(void *_param);

    void loadResetState(bool _state);

private:
    PINYMOTOR::IMotor *yaw_;

    ChassisCtrlTxPacket chassisCtrlTxPacket_;

    FILTER::LPF outFilrYaw_{ YAW_MOTOR_FREQ, 1000.f };
};

} // namespace GIMBAL
