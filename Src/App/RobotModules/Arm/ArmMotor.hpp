#pragma once

#include "DMMotor.hpp"
#include "LKMotorRS485.hpp"

#include "./ArmKinematic.hpp"
#include "./ArmSafety.hpp"

namespace ARM {

using namespace PINYMOTOR;
using namespace PINYMOTOR::DMMOTOR;

union ArmSpeed_u {
    struct {
        //没有算两个夹爪3508
        float joint1; //yaw          1   8016
        float joint2; //pitch        2   10010
        float joint3; //right pitch  3   8009
        float joint4; //right yaw    4   4340
        float joint5; //right pitch  5   3507
        float joint6; //right roll   6   3507
    };
    float _[6]; // rad/s
};

typedef union {
    struct {
        LKMOTOR::LKMotorRS485 *mgMotor1; //mg8016
        DMMOTOR::DMMotor *dmMotor2;      //10010
        DMMOTOR::DMMotor *dmMotor3;      //8009
        DMMOTOR::DMMotor *dmMotor4;      //4340
        DMMOTOR::DMMotor *dmMotor5;      //3507
        DMMOTOR::DMMotor *dmMotor6;      //3507
    };

    PINYMOTOR::IMotor *all_motors[6];
} MultiTypeMotors_t;

struct JointInfo_s {
    float angle_min;
    float angle_max;
};

class Motors {
    static constexpr float RATED_TORQUE = 0.6f; //M3507

public:
    Motors(Joint _angleMin, Joint _angleMax);
    bool init();
    void update();
    void stop();
    void enable();
    bool homing(const Joint _target_Joints);
    void ctrl(const Joint &_targetJoints);

    bool checkGoal(Joint _goal);

    bool isMoving(const Joint &_target);

    Joint currentJoints = {};
    ArmSpeed_u ref_speed = {};
    Reg_s regPm5;
    Reg_s regPm6;
    RegValue_u regPmValue5;
    RegValue_u regPmValue6;
    Safety safety;

private:
    MultiTypeMotors_t motors;
};

} // namespace ARM
