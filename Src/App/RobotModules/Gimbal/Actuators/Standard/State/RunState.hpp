#pragma once
#include "../Shooter.hpp"
#include "Standard.hpp"
#include "StmLog.hpp"
#include "Smooth.hpp"

extern uint8_t constdartId;

namespace GIMBAL {
class RunState : public FSMState<FSMState_e> {

public:
    enum class RunState_e : uint8_t { SINGLE, CONSTANT };
    RunState(Standard *_gimbal)
            : FSMState<FSMState_e>(FSMState_e::RUN), gimbal_(_gimbal)
    {
        runStateFactory_.addState(RunState_e::SINGLE,
                                  std::make_unique<SingleState>(gimbal_));
        runStateFactory_.addState(RunState_e::CONSTANT,
                                  std::make_unique<ConstState>(gimbal_));
    }//初始化，创建run下的两个状态
    void enter() final
    {
        gimbal_->enter();
        gimbal_->isLaunched = false;
        gimbal_->cmd_yawPos = gimbal_->yawState().pos;
        gimbal_->cmd_trigPos = gimbal_->trigState().pos;

        runStateFactory_.init(
                runStateFactory_.getNextState(RunState_e::SINGLE));
                //一进runStateFactory就直接进入SingleState状态机

        LOG::info("GimbalRun", "enter");
    };

    void run() final { runStateFactory_.update(); };//切换成Single

    void exit() final { LOG::info("GimbalRun", "exit"); };

    FSMState_e checkChange() final
    {
        if (gimbal_->gmsg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if(gimbal_->gmsg.state == FSMState_e::LAUNCH)
            return FSMState_e::LAUNCH;
        else
            return FSMState_e::STOP;
    };

private:
    Standard *gimbal_;
    StateFactory<RunState_e> runStateFactory_;

    class SingleState : public FSMState<RunState_e> {
    public:
        SingleState(Standard *_gimbal)
                : FSMState<RunState_e>(RunState_e::SINGLE), gimbal_(_gimbal)
        {
        }
        void enter() final
        {

        }
        void run() final
        {
//yaw
            if(gimbal_->gmsg.visFollow){
                gimbal_->cmd_yawPos = std::fmax(0.f, 
                    std::fmin((gimbal_->yawState().pos + gimbal_->vision_.data().yaw), Gimbal::angyRange));
            }
            else{
                gimbal_->cmd_yawPos = std::fmax(0.f, 
                        std::fmin((gimbal_->cmd_yawPos + gimbal_->gmsg.gainYaw), Gimbal::angyRange));     
            }
            gimbal_->yawDualAngCtrl();
//trigger
            gimbal_->cmd_trigPos = std::fmax(Gimbal::looptRange, 
                    std::fmin((gimbal_->cmd_trigPos + gimbal_->gmsg.gainTrig), 0.f));

            gimbal_->trigDualAngCtrl();
            // gimbal_->trigSingRpmCtrl(gimbal_->gmsg.gainTrig);

        }
        void exit() final { LOG::info("GimbalRun", " Normal exit"); }
        RunState_e checkChange() final
        {
            if (gimbal_->gmsg.isConstant)
                return RunState_e::CONSTANT;
            else
                return RunState_e::SINGLE;
            return RunState_e::SINGLE;
        }

    private:
        Standard *gimbal_;
    };

    class ConstState : public FSMState<RunState_e> {
    public:
        ConstState(Standard *_gimbal)
                : FSMState<RunState_e>(RunState_e::CONSTANT), gimbal_(_gimbal)
        {
        }
        void enter() final
        {
            constdartId = 0;
            gimbal_->enter();
        }
        void run() final
        {
            if(constdartId < 4){                  
                gimbal_->cmd_yawPos = std::fmax(0.f, 
                        std::fmin((gimbal_->yawState().pos + gimbal_->vision_.data().yaw), Gimbal::angyRange));

                gimbal_->cmd_trigPos = gimbal_->constangTrig[constdartId];

                if(std::fabs(gimbal_->yawState().pos - gimbal_->cmd_yawPos) <
                                Gimbal::YAW_LAUNCH_ANG_DEADBAND &&
                    std::fabs(gimbal_->trigState().pos - gimbal_->constangTrig[constdartId]) <
                                Gimbal::TRIG_LAUNCH_ANG_DEADBAND){
                                    constloadedGimbal = true;
                }
                else{
                    constloadedGimbal = false;
                }
                
                gimbal_->yawDualAngCtrl();
                gimbal_->trigDualAngCtrl();
            }
            else{
                ;
            }

        }
        void exit() final {
            constloadedGimbal = false;
        };

        RunState_e checkChange() final
        {
            if (gimbal_->gmsg.isConstant)
                return RunState_e::CONSTANT;
            else
                return RunState_e::SINGLE;
            return RunState_e::CONSTANT;
        }

    private:
        Standard *gimbal_;

    };
};
} // namespace GIMBAL