#pragma once
#include "IMotor.hpp"
#include "ArmKinematic.hpp"
#include "StmLog.hpp"
#include <cstdint>


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
    float _[7]; // rpm
};


class Motors {
public:
    Motors();
    void init();
    void update();
    void stop();
    void ctrl(const Joint7D &_target_joints);

    LOG::Logger &log = LOG::Logger::instance();
    enum class JointState_e : uint8_t { FINISH_STATE = 0, MOVING_STATE };
    JointState_e jointStateFlag = JointState_e::FINISH_STATE;

    struct MotionState_s {
        int16_t rateCnt = 0;        // 速度变化率计数器
        float unitreeAngleFix = 0;  // 宇树电机角度补偿
        float unitreeInitAngle = 0; // 宇树电机初始角度
        float jointSpeed = 0.8f;    // 关节速度限制
        uint8_t pointCnt = 0;       // 点位计数器
        Joint7D targetJoints;       // 目标关节位置缓存
    };
    Motors::JointState_e moveOneGoal(const Joint7D &_goal);
    struct JointInfo_s {
        float angle_min;
        float angle_max;
    };
    JointInfo_s jointInfos[7];

    bool homingUT();
    bool checkGoal(Joint7D _goal);

    ArmSpeed_u ref_speed;

protected:
    void biasJoint3Angle();
    float joint3HighPoint(float _target);
    float joint3LowPoint(float _target);

private:
    std::unique_ptr<PINYMOTOR::IMotor> motor[7];
    Joint7D current_joints;
    Joint7D target_joints; //two

    MotionState_s motionState;
};

} // namespace ARM
