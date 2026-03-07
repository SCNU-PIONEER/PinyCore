#pragma once

#include "Chassis.hpp"

#include "IMotor.hpp"
#include "IIR.hpp"

#include "RxPacket.hpp"
#include "PacketImpl.hpp"
#include "QuadricycleController.hpp"

namespace CHASSIS {

class ChassisCtrlRxPacket : public COMM::RxPacket<COMM::ChassisCtrlPacketType, 1> {
public:
    ChassisCtrlRxPacket() { registerCallback(); }
    void registerCallback() final;
};

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

class Mecanum : public Chassis {
public:
    static constexpr float W_DIAMETER = 0.1525f;
    static constexpr float W_CIRCUMFERENCE = (M_PI * W_DIAMETER);
    static constexpr float FRONT_R = 0.354f;
    static constexpr float BACK_R = 0.354f;

    Mecanum();

    void stop();
    void enter();
    void ctrl(const Speed_u &_speed);
    void update();
    const Speed_u &getSpeed() const { return curSpeed_; }

protected:
    /*
     * @brief: calculate the speed of chassis
     * @param: _wSpeed: speed of each wheel
     * @return: speed of the chassis in m/s and rad/s
    */
    Speed_u forward(const WheelsSpeed_u &_wSpeed);

    /*
     * @brief: calculate the speed of each wheel
     * @param: _speed: speed of the chassis in m/s and rad/s
     * @return: speed of each wheel
    */
    WheelsSpeed_u reverse(const Speed_u &_speed);

private:
    Motors_u motors_ = {};

#ifdef TARGET_SLAVE
    ChassisCtrlRxPacket chassisCtrlRxPacket_;
#endif

    QuadricycleController *powerCtrl_;
    WheelsSpeed_u wSpeed_ = {}; // 当前轮速，单位 rpm
    Speed_u curSpeed_ = {};     // 当前底盘速度，单位 m/s 和 rad/s
    FILTER::LPFIIR3 iir3_[4];   // 轮速滤波器
};

} //namespace CHASSIS
