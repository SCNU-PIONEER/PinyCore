/*
 * @Author: IRON.NY 2537933690@qq.com
 * @Date: 2025-12-12 16:07:55
 * @LastEditors: IRON.NY 2537933690@qq.com
 * @LastEditTime: 2026-03-07 17:38:50
 * @FilePath: \PinyCore\Src\App\RobotModules\Gimbal\Actuators\Standard\State\StopState.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "Standard.hpp"
#include "StmLog.hpp"

namespace GIMBAL {

class StopState : public FSMState<FSMState_e> {
public:
    StopState(Standard *_gimbal) : FSMState<FSMState_e>(FSMState_e::STOP), gimbal_(_gimbal) {}

    void enter() final
    {
        gimbal_->isLaunched = false;
        gimbal_->stop();
        LOG::info("GimbalStop", "enter");
    }

    void run() final { gimbal_->stop(); }

    void exit() final { LOG::info("GimbalStop", "exit"); }

    FSMState_e checkChange() final
    {
        if (gimbal_->gmsg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (gimbal_->gmsg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else
            return (FSMState_e)0;
    }

private:
    Standard *gimbal_;
};

} // namespace GIMBAL
