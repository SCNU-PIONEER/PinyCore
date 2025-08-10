#pragma once
#include <cmath>
#include "Locomotion.hpp"

namespace CHASSIS {

union WheelsSpeed_u {
    struct {
        //GM6020s
        float S_RF;
        float S_LF;
        float S_LB;
        float S_RB;
        //M3508
        float M_RF;
        float M_LF;
        float M_LB;
        float M_RB;
    };
    float _[8]; // rpm
};

typedef union {
    struct {
        float DirAng_RF; //rad
        float DirAng_LF; //rad
        float DirAng_LB; //rad
        float DirAng_RB; //rad
    };
    float _[4]; //
} SteerDirectionAngle_u;

//喵板
class Steer : public Locomotion {
public:
    static constexpr float W_FRONT_REAR = 0.35f; //前后轮距
    static constexpr float W_LEFT_RIGHT = 0.35f; //左右轮距
    static constexpr float W_DIAMETER = 0.005f;  //轮子直径
    static constexpr float W_CIRCUMFERENCE = (M_PI * W_DIAMETER);
    static constexpr float CAR_RADIUS = 0.f; //车体旋转半径
    static constexpr float W_CDISTANCE = 0.f;

    Steer();
    // ~Steer() = default;

    void stop() override;
    void enter() override;
    void update() override;
    void ctrl(const Speed_u &_refSpeed) override;

protected:
    //舵轮零位角度，用于回正方向
    float SteerZero[4] = { 0, 0, 0, 0 };

    SteerDirectionAngle_u curDir; //舵轮当前方向角
    
    SteerDirectionAngle_u expDir; //舵轮期望方向角

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
    PINYMOTOR::IMotor *motor_[8];
    WheelsSpeed_u wSpeed_;
};

} //namespace CHASSIS
