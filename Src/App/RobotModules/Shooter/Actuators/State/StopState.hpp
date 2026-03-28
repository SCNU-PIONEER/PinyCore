/*
 * @Author: IRON.NY 2537933690@qq.com
 * @Date: 2025-12-12 16:07:55
 * @LastEditors: IRON.NY 2537933690@qq.com
 * @LastEditTime: 2026-03-12 02:39:30
 * @FilePath: \PinyCore\Src\App\RobotModules\Gimbal\Actuators\Standard\State\StopState.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "../Leather.hpp"
#include "StmLog.hpp"

namespace SHOOTER {

class StopState : public FSMState<FSMState_e> {
public:
    StopState(Leather *_shooter) : FSMState<FSMState_e>(FSMState_e::STOP), shooter_(_shooter) {}

    void enter() final
    {
        shooter_->isLaunched = false;
        shooter_->stop();
        LOG::info("ShooterStop", "enter");
    }

    void run() final { shooter_->stop(); }

    void exit() final { LOG::info("ShooterStop", "exit"); }

    FSMState_e checkChange() final
    {
        if(shooter_->smsg.state == FSMState_e::STOP){
            return FSMState_e::STOP;
        }
        else if(shooter_->smsg.state == FSMState_e::RUN){
            return FSMState_e::RUN;
        }
        return FSMState_e::STOP;
    }

private:
    Leather *shooter_;
};

} // namespace SHOOTER
