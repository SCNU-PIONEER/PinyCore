#pragma once

#include "Chassis.hpp"

#include "sdkconfig.h"

#include "IMotor.hpp"
#include "dsp/fast_math_functions.h"

#include "IIR.hpp"

#include "RxPacket.hpp"
#include "PacketImpl.hpp"

namespace CHASSIS {

union Speed_u {
    struct {
        float vx; // m/s
        float vy; // m/s
        float wz; // rad/s
    };
    float _[3];
};
union WheelsSpeed_u {
    struct {
        float M_RF; // motor of the right front
        float M_LF; // motor of the left front
        float M_LB; // motor of the left back
        float M_RB; // motor of the right back
    };
    float _[4]; // rpm
};

union Motors_u {
    struct {
        PINYMOTOR::IMotor *RF;
        PINYMOTOR::IMotor *LF;
        PINYMOTOR::IMotor *LB;
        PINYMOTOR::IMotor *RB;
    };
    PINYMOTOR::IMotor *_[4];
};

class ChassisCtrlRxPacket : public COMM::RxPacket<COMM::SamplePacketType, 1> {
public:
    // TODO: slightly cumbersome like this or CRTP
    ChassisCtrlRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class Omni : public Chassis {
public:
    static constexpr float W_DIAMETER = 0.1625f;
    static constexpr float W_RADIUS = (W_DIAMETER / 2.f);
    static constexpr float W_CIRCUMFERENCE = (PI * W_DIAMETER);
    static constexpr float CHASSIS_R = 0.24911f;

    static constexpr float MAX_VX_SPEED = 2.f;
    static constexpr float MAX_VY_SPEED = 2.f;
    static constexpr float MAX_WZ_SPEED = 3.f;

    Omni();

    void stop();
    void enter();
    void update(void *_param);
    void ctrl(const Speed_u &_speed);

    void loadDeltaYaw();
    bool isGimbalReset() const;

    const Speed_u &getSpeed() const { return curSpeed_; }


    float deltaYaw = 0.f;

private:
    /**
     * @brief : calculate the speed of the chassis
     * 
     * @param _wSpeed : rpm of each wheel
     * @return Speed_u : speed of the chassis in m/s and rad/s
     */
    Speed_u forward(const WheelsSpeed_u &_wSpeed);

    /**
     * @brief : calculate the speed of each wheel
     * 
     * @param _speed : speed of the chassis in m/s and rad/s
     * @return WheelsSpeed_u : radps of each wheel
     */
    WheelsSpeed_u reverse(const Speed_u &_speed);

    Motors_u motors_;
    WheelsSpeed_u wSpeed_;
    Speed_u curSpeed_;

    FILTER::IIR3 velFltr_[4];

#if !APP_USE_COMM
    GimbalYawMsg_s gimbalYawMsg_;
    Subscriber<GimbalYawMsg_s> *gimbalYawSub_;
#endif
#ifdef APP_CHASSIS_SLAVE
    ChassisCtrlRxPacket chassisCtrlRxPacket_;
    Subscriber<INS_SYS::INSData_s> *insSub_;
#endif

    float cosYaw_ = 0.f;
    float sinYaw_ = 0.f;
};

} //namespace CHASSIS
