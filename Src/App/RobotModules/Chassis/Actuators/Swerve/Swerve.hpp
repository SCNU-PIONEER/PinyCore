#pragma once
#include "Chassis.hpp"

#include "IMotor.hpp"
#include "Pid.hpp"
#include "PidBasic.hpp"
#include "dsp/fast_math_functions.h"

#include "IIR.hpp"
#include "LPF.hpp"

namespace CHASSIS::SWERVE {
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
        float M_LF; // motor of the left front
        float M_RF; // motor of the right front
        float M_RB; // motor of the right back
        float M_LB; // motor of the left back
    };
    float _[4]; // rpm
};

union Motors_u {
    struct {
        PINYMOTOR::IMotor *M_LF;
        PINYMOTOR::IMotor *M_RF;
        PINYMOTOR::IMotor *M_RB;
        PINYMOTOR::IMotor *M_LB;

        PINYMOTOR::IMotor *S_LF;
        PINYMOTOR::IMotor *S_RF;
        PINYMOTOR::IMotor *S_RB;
        PINYMOTOR::IMotor *S_LB;
    };
    PINYMOTOR::IMotor *_[8];
};

struct SwervePosPid_s {
    PID *posPID;
    PID *velPID;
};
union SwervePid_u {
    struct {
        SwervePosPid_s *S_LF;
        SwervePosPid_s *S_RF;
        SwervePosPid_s *S_RB;
        SwervePosPid_s *S_LB;
    };
    SwervePosPid_s *_[4];
};

typedef union {
    struct {
        float DirAng_LF; //rad
        float DirAng_RF; //rad
        float DirAng_RB; //rad
        float DirAng_LB; //rad
    };
    float _[4]; //
} SwerveDirectionAngle_u;


typedef union {
    struct {
        FILTER::LPF *LF;
        FILTER::LPF *RF;
        FILTER::LPF *RB;
        FILTER::LPF *LB;
    };
    FILTER::LPF *_[4];
} SwerveFilterOutput_u;

//喵板
class Swerve : public Chassis<Swerve> {
public:
    static constexpr float W_FRONT_REAR = 0.298f; //前后轮距/m
    static constexpr float W_LEFT_RIGHT = 0.298f; //左右轮距
    static constexpr float W_DIAMETER = 0.098f;   //轮子直径
    static constexpr float W_CIRCUMFERENCE = (M_PI * W_DIAMETER);
    static constexpr float CAR_RADIUS = 0.210f; //车体旋转半径

    Swerve();

    //LF,RF,RB,LB
    SwerveFilterOutput_u SwerveFilterOutput_;

    void stopSelf();
    void enterSelf();
    void updateSelf();

    void ctrl(const Speed_u &_refSpeed);

    SwervePid_u swervePid_;

    SwerveDirectionAngle_u curDir; //舵轮当前方向角
    SwerveDirectionAngle_u expDir; //舵轮期望方向角



protected:
    //舵轮零位角度，用于回正方向,rad
    float SwerveZero[4] = { 3.6206367f, 0.556135f, 4.008f, 3.03151608f };
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
    Motors_u motors_;
    WheelsSpeed_u wSpeed_;
    Speed_u curSpeed_;

    friend class Chassis<Swerve>;
};

} // namespace CHASSIS::SWERVE
