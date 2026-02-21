#pragma once
#include "DMMotor.hpp"
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

        float joint3; //pitch  3   8009
        float joint4; //yaw    4   4340
        float joint5; //pitch  5   3507
        float joint6; //roll   6   3507

        float joint7;  //pitch  7   8009
        float joint8;  //yaw    8   4340
        float joint9;  //pitch  9   3507
        float joint10; //roll   10  3507
    };
    float _[10]; // rad/s
};

typedef union {
    struct {
        DMMOTOR::DMMotor *mgMotor1;  //mg8016
        DMMOTOR::DMMotor *dmMotor2;  //10010
        DMMOTOR::DMMotor *dmMotor3;  //8009
        DMMOTOR::DMMotor *dmMotor4;  //4340
        DMMOTOR::DMMotor *dmMotor5;  //3507
        DMMOTOR::DMMotor *dmMotor6;  //3507
        DMMOTOR::DMMotor *dmMotor7;  //8009
        DMMOTOR::DMMotor *dmMotor8;  //4340
        DMMOTOR::DMMotor *dmMotor9;  //3507
        DMMOTOR::DMMotor *dmMotor10; //3507
    };

    PINYMOTOR::IMotor *all_motors[10];
} MultiTypeMotors_t;

struct JointInfo_s {
    float angle_min;
    float angle_max;
};

class Motors {
public:
    Motors(Joint10 _angleMin, Joint10 _angleMax);
    bool init();
    void update();
    void stop();
    void enable();
    void ctrl(const Joint10 &_targetJoints);

    bool checkGoal(Joint _goal);

    bool isMoving(const Joint10 &_target);

    JointInfo_s jointInfos[10];
    Joint10 currentJoints = {};
    ArmSpeed_u ref_speed = {};
    Safety safety;

private:
    MultiTypeMotors_t motors;
};

} // namespace ARM
