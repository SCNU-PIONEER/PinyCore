#pragma once
#include "Locomotion.hpp"
#include "FSMState.hpp"
#include "MsgImpl.hpp"
#include "StmLog.hpp"

namespace GIMBAL {
    //state machine states
enum class CtrlType : uint8_t { 
    USE_MOTOR = 0, 
    USE_IMU 
};
}

class Gimbal {
public:
 

    void update(); // 主更新函数
    
    void transitionTo(GIMBAL::FSMState_e newState);

    // 云台核心参数
    struct Params {
    static constexpr  float pitchRR = 1.0f;
    static constexpr  float yawRR = 1.0f;
    
    static constexpr  float pitch_Min = 0.1f;        
    static constexpr float pitch_Max = 0.77539f;
    static constexpr float yaw_Min = 0.7198f;
    static constexpr float yaw_Max = 5.0945f;
    static constexpr float softStartDeadband = 0.062f;

    static constexpr  float pitchCenter = 0;
    static constexpr   float yawCenter = 0;
    };
    
   
    
    // 成员变量
    Params params;
    
   
    
    // 控制方法
    void smoothReset();
    void clearState();
    void motorCtrl();
  
    



};