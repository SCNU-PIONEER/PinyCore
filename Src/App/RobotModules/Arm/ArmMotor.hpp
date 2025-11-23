#pragma once
#include "IMotor.hpp"
#include "DMMotor.hpp"
#include "MotorManager.hpp"
#include "./ArmKinematic.hpp"
#include "./ArmSafety.hpp"

namespace ARM {

using namespace PINYMOTOR;

union ArmSpeed_u {
    struct {
        float joint1;
        float joint2;
        float joint3;
        float joint4;
        float joint5;
        float joint6;
    };
    float _[6]; // rad/s
};

typedef union {
    struct {
        DMMOTOR::DMMotor *dmMotor1;
        DMMOTOR::DMMotor *dmMotor2;
        DMMOTOR::DMMotor *dmMotor3;
        DMMOTOR::DMMotor *dmMotor4;
        DMMOTOR::DMMotor *dmMotor5;
        DMMOTOR::DMMotor *dmMotor6;
    };

    PINYMOTOR::IMotor *all_motors[6];
} MultiTypeMotors_t;


class Motors {
public:
    Motors();
    bool init();
    void update();
    void stop();
    void enable();
    void ctrl(const Joint &_target_joints);


    bool checkGoal(Joint _goal);


    Joint currentJoints;
    Joint angleMin; // motor min angle
    Joint angleMax; // motor min angle

    Safety safety;

private:
    MultiTypeMotors_t motors;
};

} // namespace ARM
