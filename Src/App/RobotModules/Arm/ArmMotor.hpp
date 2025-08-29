#pragma once
#include "DJIMotor.hpp"
#include "IMotor.hpp"
#include "ArmKinematic.hpp"
#include "StmLog.hpp"
#include "UTMotor.hpp"
#include "DMMotor.hpp"
#include "MotorManager.hpp"


namespace ARM {

union ArmSpeed_u {
    struct {
        float joint1;
        float joint2;
        float joint3;
        float joint4;
        float joint5;
        float joint6;
        float joint7;
    };
    float _[7]; // rad/s
};

typedef union {
    struct {
        PINYMOTOR::UTMOTOR::UTMotor *utMotor;
        PINYMOTOR::DMMOTOR::DMMotor *dmMotor1;
        PINYMOTOR::DMMOTOR::DMMotor *dmMotor2;
        PINYMOTOR::DMMOTOR::DMMotor *dmMotor3;
        PINYMOTOR::DMMOTOR::DMMotor *dmMotor4;
        PINYMOTOR::DMMOTOR::DMMotor *dmMotor5;
        PINYMOTOR::DJIMOTOR::DJIMotor *djMotor;
    };

    // 基类指针数组（用于批量操作）
    PINYMOTOR::IMotor *all_motors[7];
} MultiTypeMotors_t;

class Motors {
    static constexpr float UNITREE_KP = 4.8;
    static constexpr float UNITREE_KD = 0.02;

public:
    Motors();
    void init();
    void update();
    void stop();
    void ctrl(const Joint7D &_target_joints);

    LOG::Logger &log = LOG::Logger::instance();

    struct JointInfo_s {
        float angle_min;
        float angle_max;
    };
    JointInfo_s jointInfos[7];

    bool homingUT();
    bool checkGoal(Joint7D _goal);
    void setUTsmoothStart();

    ArmSpeed_u ref_speed;
    Joint7D current_joints;

    void biasJoint3Angle();
    float joint3HighPoint(float _target);
    float joint3LowPoint(float _target);

private:
    MultiTypeMotors_t motors;

    float unitreeAngleFix = 0; // 宇树电机角度补偿
};

} // namespace ARM
