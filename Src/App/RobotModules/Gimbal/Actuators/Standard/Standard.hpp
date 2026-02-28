#pragma once

#include "Gimbal.hpp"
#include "IMotor.hpp"
#include "TxPacket.hpp"
#include "PacketImpl.hpp"

namespace GIMBAL {


class ChassisCtrlTxPacket : public COMM::TxPacket<COMM::ChassisCtrlPacketType> {
    friend class Standard;
    void transmit(uint8_t *_buf, uint16_t _len) final;
};

class Standard : public Gimbal {
public:
    static constexpr float YAW_LAUNCH_ANG = 4.74211359f;
    static constexpr float YAW_MOTOR_FREQ = 500.f;
    static constexpr float YAW_SPEED_MAX = 1.f;
    // "REVERSE flag" is true when imu and motor have different direction
    static constexpr bool YAW_REVERSE = true;

    Standard();

    PINYMOTOR::IMotor *motor() const;

    void stop();
    void enter();
    void update(void *_param);
    void ctrl(float _pos, float _maxSpeed = YAW_SPEED_MAX);

    void loadResetState(bool _state);

private:
    PINYMOTOR::IMotor *motor_;
    ChassisCtrlTxPacket chassisCtrlTxPacket_;
};

} // namespace GIMBAL
