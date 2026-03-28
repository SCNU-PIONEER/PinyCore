/*
 * @Author: IRON.NY 2537933690@qq.com
 * @Date: 2025-12-12 16:07:55
 * @LastEditors: IRON.NY 2537933690@qq.com
 * @LastEditTime: 2026-03-23 16:59:20
 * @FilePath: \PinyCore\Src\App\RobotModules\Gimbal\Actuators\Standard\State\LaunchState.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "FSMState.hpp"
#include "MotorCommonMacros.hpp"
#include "Standard.hpp"

namespace GIMBAL {
class LaunchState : public FSMState<FSMState_e> {
    static constexpr uint16_t MAX_LAUNCH_TIME = 20000; // 20s

public:
    LaunchState(Standard *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::LAUNCH), gimbal_(_gimbal)
    {
    }

    void enter() final
    {
        launchTime_ = 0;
        // launchFinishTime_ = 0;
        gimbal_->isLaunched = false;
        gimbal_->enter();
        LOG::info("GimbalLaunch", "enter");
    };

    void run() final
    {
        if (launchTime_ < MAX_LAUNCH_TIME) {
            launchTime_++;
            if(gimbal_->alreadyLaunched){//再次缓启动
                gimbal_->cmd_yawPos = 0.f;//云台归零
                gimbal_->cmd_trigPos = 0.f;
                
                if (std::fabs(gimbal_->yawState().pos - gimbal_->cmd_yawPos) <
                            Gimbal::YAW_LAUNCH_ANG_DEADBAND &&
                        (std::fabs(gimbal_->trigState().pos - gimbal_->cmd_trigPos) <
                            Gimbal::TRIG_LAUNCH_ANG_DEADBAND)) {
                    gimbal_->isLaunched = true;
                }  
                gimbal_->yawDualAngCtrl();
                gimbal_->trigDualAngCtrl();
            }
            else{//首次缓启动
                float expyRpm = 0;
                float exptRpm = 0;
                if(gimbal_->yawLaunched & gimbal_->trigLaunched){
                    gimbal_->alreadyLaunched = true;
                    gimbal_->isLaunched = true;
                }
                else{

                    if(HAL_GPIO_ReadPin(GPIO_YAW_GPIO_Port, GPIO_YAW_Pin) == GPIO_PIN_SET){
                        gimbal_->yawLaunched = true;  
                        gimbal_->getyZeroPos();
                    }
                    if(HAL_GPIO_ReadPin(GPIO_TRIG_GPIO_Port, GPIO_TRIG_Pin) == GPIO_PIN_SET){
                        gimbal_->trigLaunched = true;
                        gimbal_->gettZeroPos();
                    }

                    if(gimbal_->yawLaunched)
                        expyRpm = 0;
                    else
                        expyRpm = -180.f;
                    
                    if(gimbal_->trigLaunched)
                        exptRpm = 0;
                    else
                        exptRpm = 5500.f;
                }
                
                gimbal_->yawSingRpmCtrl(expyRpm);
                gimbal_->trigSingRpmCtrl(exptRpm);
            }

        } else {
            gimbal_->isLaunched = true;
        }
    };

    void exit() final
    {
        gimbal_->stop();
        gimbal_->cmd_yawPos = 0.f;
        LOG::info("GimbalLaunch", "exit");
    };

    FSMState_e checkChange() final
    {
        if(gimbal_->gmsg.state != FSMState_e::STOP){
            if(gimbal_->isLaunched)
                return FSMState_e::RUN;
            else
                return FSMState_e::LAUNCH;
        }
        else{
            return FSMState_e::STOP;
        }
        return FSMState_e::LAUNCH;
    }

private:
    Standard *gimbal_;

    uint16_t launchTime_ = 0;
    uint16_t launchFinishTime_ = 0;
};
} // namespace GIMBAL::STANDARD
