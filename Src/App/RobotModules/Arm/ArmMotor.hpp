#pragma once
#include "DMMotor.hpp"
#include "LKMotorRS485.hpp"
#include "./ArmKinematic.hpp"
#include "./ArmSafety.hpp"
#include "ArmPlan.hpp"

namespace ARM {

using namespace PINYMOTOR;

union ArmSpeed_u {
    struct {
        //没有算两个夹爪3508
        float joint1; //yaw    1   8016
        float joint2; //pitch  2   10010

        float joint3; //right pitch  3   8009
        float joint4; //right yaw    4   4340
        float joint5; //right pitch  5   3507
        float joint6; //right roll   6   3507

        float joint7;  //left pitch  7   8009
        float joint8;  //left yaw    8   4340
        float joint9;  //left pitch  9   3507
        float joint10; //left roll   10  3507
    };
    float _[10]; // rad/s
};

typedef union {
    struct {
        LKMOTOR::LKMotorRS485 *mgMotor1; //mg8016
        DMMOTOR::DMMotor *dmMotor2;      //10010
        DMMOTOR::DMMotor *dmMotor3;      //8009
        DMMOTOR::DMMotor *dmMotor4;      //4340
        DMMOTOR::DMMotor *dmMotor5;      //3507
        DMMOTOR::DMMotor *dmMotor6;      //3507
        DMMOTOR::DMMotor *dmMotor7;      //8009
        DMMOTOR::DMMotor *dmMotor8;      //4340
        DMMOTOR::DMMotor *dmMotor9;      //3507
        DMMOTOR::DMMotor *dmMotor10;     //3507
    };

    PINYMOTOR::IMotor *all_motors[10];
} MultiTypeMotors_t;

struct JointInfo_s {
    float angle_min;
    float angle_max;
};

class Motors {
    static constexpr float RATED_TORQUE = 0.4f; //M3507

public:
    Motors(Joint10 _angleMin, Joint10 _angleMax);
    bool init();
    void update();
    void stop();
    void enable();
    void ctrl(const Joint10 &_targetJoints);

    bool checkGoal(Joint _goal);

    bool isMoving(const Joint10 &_target);

    void handRollPitch(Joint10 &_targetJoints, Joint10 &_msgTargetJoints);

    Joint10 currentJoints = {};
    ArmSpeed_u ref_speed = {};
    Safety safety;

private:
    MultiTypeMotors_t motors;
};

} // namespace ARM
