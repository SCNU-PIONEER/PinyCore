/*
 * @Author: IRON.NY 2537933690@qq.com
 * @Date: 2025-12-12 16:07:55
 * @LastEditors: IRON.NY 2537933690@qq.com
 * @LastEditTime: 2026-03-27 21:40:49
 * @FilePath: \PinyCore\Src\App\RobotModules\Gimbal\Actuators\Standard\State\LaunchState.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "FSMState.hpp"
#include "MotorCommonMacros.hpp"
#include "../Leather.hpp"

namespace SHOOTER {
class LaunchState : public FSMState<FSMState_e> {
    static constexpr uint16_t MAX_LAUNCH_TIME = 20000; // 20s


public:
    LaunchState(Leather *_shooter)
            : FSMState<FSMState_e>(FSMState_e::LAUNCH), shooter_(_shooter)
    {
    }

    void enter() final
    {
        launchTime_ = 0;
        // launchFinishTime_ = 0;
        shooter_->isLaunched = false;
        shooter_->enter();
        LOG::info("ShooterLaunch", "enter");
    };

    void run() final
    {
        if(shooter_->alreadyLaunched){
            shooter_->cmd_screwPos = -1.f;
            if(fabsf(shooter_->screwState().pos - shooter_->cmd_screwPos) < 1.f)
                shooter_->isLaunched = true;
            else
                shooter_->isLaunched = false;
            shooter_->screwDualAngCtrl();
        }
        else{
            float expsRpm = 23.f;
            if((fabsf(shooter_->shooterL()->data().torq) + fabsf(shooter_->shooterR()->data().torq)) / 2.f 
                    > 0.25f){
                shooter_->isLaunched = true;
                shooter_->alreadyLaunched = true;
                shooter_->getsZeroPos();
                expsRpm = 0.f;
            }
            shooter_->screwSingRpmCtrl(expsRpm);
        }

    };

    void exit() final
    {
        shooter_->stop();
        shooter_->cmd_screwPos = 0.f;
        LOG::info("GimbalLaunch", "exit");
    };

    FSMState_e checkChange() final
    {
        if(shooter_->smsg.state != FSMState_e::STOP){
            if(shooter_->isLaunched)
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
    Leather *shooter_;

    uint16_t launchTime_ = 0;
    uint16_t launchFinishTime_ = 0;
};
} // namespace SHOOTER::STANDARD
