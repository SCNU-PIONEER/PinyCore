#pragma once
#include "../Gimbal/Gimbal.hpp"
#include "../Leather.hpp"
#include "StmLog.hpp"
#include "Smooth.hpp"
#include "../Bsp/Can/Bsp_can.hpp"

extern bool constloadedGimbal;
extern uint8_t constdartId;

namespace SHOOTER {
class RunState : public FSMState<FSMState_e> {

public:
    enum class RunState_e : uint8_t { SINGLE, CONSTANT };//SINGLE和CONSTANT两个状态
    RunState(Leather *_shooter)
            : FSMState<FSMState_e>(FSMState_e::RUN), shooter_(_shooter)
    {
        runStateFactory_.addState(RunState_e::SINGLE,
                                  std::make_unique<SingleState>(shooter_));
        runStateFactory_.addState(RunState_e::CONSTANT,
                                  std::make_unique<ConstState>(shooter_));
    }//初始化，创建run下的两个状态
    void enter() final
    {
        shooter_->enter();
        runStateFactory_.init(
                runStateFactory_.getNextState(RunState_e::SINGLE));

        LOG::info("ShooterRun", "enter");
    };

    void run() final { runStateFactory_.update(); };//在此处切换single或者constant

    void exit() final { LOG::info("ShooterRun", "exit"); };

    FSMState_e checkChange() final
    {
        if(shooter_->smsg.state == FSMState_e::RUN){
            return FSMState_e::RUN;
        }
        else if(shooter_->smsg.state == FSMState_e::LAUNCH){
            return FSMState_e::LAUNCH;
        }
        else{
            return FSMState_e::STOP;
        }
    };

private:
    Leather *shooter_;
    StateFactory<RunState_e> runStateFactory_;

    class SingleState : public FSMState<RunState_e> {
    public:
        SingleState(Leather *_shooter)
                : FSMState<RunState_e>(RunState_e::SINGLE), shooter_(_shooter)
        {
        }
        void enter() final
        {

        }
        void run() final
        {
            // shooter_->cmd_screwPos = std::fmax(Shooter::loopsRange, 
            //         std::fmin((shooter_->cmd_screwPos + shooter_->smsg.gainScrew), 0.f));
            shooter_->cmd_screwPos = shooter_->cmd_screwPos + shooter_->smsg.gainScrew;
            shooter_->cmd_turnWheelPos = Shooter::angTurnwheel_Load[shooter_->smsg.turnId];
           
            shooter_->screwDualAngCtrl();
            shooter_->turnWheelCtrl();

            if(shooter_->smsg.fireFlag){
                shooter_->fireFlag = true;
            }
            else{
                shooter_->fireFlag = false;
            }
            shooter_->shooterServoCtrl();

        }
        void exit() final { LOG::info("ShooterRun", " Normal exit"); }
        RunState_e checkChange() final
        {
            if(shooter_->smsg.isConstant){
                return RunState_e::CONSTANT;
            }
            else{
                return RunState_e::SINGLE;
            }
            return RunState_e::CONSTANT;
        }

    private:
        Leather *shooter_;
    };

    class ConstState : public FSMState<RunState_e> {
    public:
        ConstState(Leather *_shooter)
                : FSMState<RunState_e>(RunState_e::CONSTANT), shooter_(_shooter)
        {
        }
        void enter() final
        {
            constdartId = 0;
            shooter_->dartLoaded = false;
            shooter_->screwBack = true;
            shooter_->fireFlag = false;
        }
        void run() final
        {
            if(constdartId < 4){
                if(shooter_->screwBack == true && shooter_->dartLoaded == false){
                    shooter_->cmd_screwPos = Shooter::loopsRange;
                    shooter_->cmd_turnWheelPos = Shooter::angTurnwheel_reLoad[constdartId];
                    shooter_->servoFlag[constdartId] = false;//下来的时候，舵机保持在上面
                    shooter_->magFlag[constdartId] = false;//磁铁保持吸力
                    // if((fabsf(shooter_->shooterL()->torq()) + fabsf(shooter_->shooterR()->torq())) / 2.f > 0.14 &&
                    //      fabsf(shooter_->screwState().pos - Shooter::loopsRange) < 5.f &&
                    //         fabsf(shooter_->turnwheelState().pos - Shooter::angTurnwheel_Load[constdartId]) < 0.018f){

                    if(fabsf(shooter_->screwState().pos - Shooter::loopsRange) < 4.f &&
                        fabsf(PINYMOTOR::getMinorArc(shooter_->turnwheelState().pos, Shooter::angTurnwheel_reLoad[constdartId])) < 0.1f){    
                        shooter_->dartLoaded = true;
                        shooter_->screwBack = false;
                        shooter_->magFlag[constdartId] = false;
                        shooter_->servoFlag[constdartId] = false;//舵机保持
                        shooter_->cmd_turnWheelPos = Shooter::angTurnwheel_Load[constdartId];//换弹
                    }
                }
                else if(shooter_->screwBack == false && shooter_->dartLoaded == true){
                    shooter_->cmd_screwPos = -2.f;
                    static uint16_t cnt = 0;
                    if(fabsf(PINYMOTOR::getMinorArc(shooter_->turnwheelState().pos, Shooter::angTurnwheel_Load[constdartId])) < 0.018f){
                        cnt++;
                        if(cnt < 1500){//留1.5s时间舵机下来
                            shooter_->servoFlag[constdartId] = true;
                        }
                        else if(cnt >= 1500 && cnt < 4500){//留3s时间电磁铁断磁,时间不能超过1min
                            shooter_->servoFlag[constdartId] = true;
                            shooter_->magFlag[constdartId] = true;
                        }
                        else if(cnt >= 4500 && cnt < 6000){//舵机先不归位
                            shooter_->servoFlag[constdartId] = true;
                            shooter_->magFlag[constdartId] = false;
                        }
                        else if(cnt >= 6000){
                            shooter_->cmd_turnWheelPos = Shooter::angTurnwheel_reLoad[constdartId];
                        }
                    }

                    if(fabsf(shooter_->screwState().pos - shooter_->cmd_screwPos) < 0.5f &&
                            fabsf(PINYMOTOR::getMinorArc(shooter_->turnwheelState().pos, Shooter::angTurnwheel_reLoad[constdartId])) < 0.1f){
                        shooter_->screwBack = true;
                        cnt = 0;
                    }
                }
                else if(shooter_->screwBack == true && shooter_->dartLoaded == true){
                    shooter_->fireFlag = true;
                    static uint16_t cnt = 0;
                    cnt++;
                    if(cnt < 2000)//软件延时2s
                        cnt++;
                    else{
                        shooter_->dartLoaded = false;
                        shooter_->screwBack = true;
                        constdartId++;
                        shooter_->fireFlag = false;
                        cnt = 0;
                    }
                }

                shooter_->screwDualAngCtrl();
                shooter_->turnWheelCtrl();
                shooter_->shooterServoCtrl();
            }
            else{
                ;
            }
        }
        void exit() final { 
            shooter_->fireFlag = false;
        }

        RunState_e checkChange() final
        {
            if(shooter_->smsg.isConstant){
                return RunState_e::CONSTANT;
            }
            else{
                return RunState_e::SINGLE;
            }
            return RunState_e::CONSTANT;
        }

    private:
        Leather *shooter_;
        float turnAng_ = 0.f;

    };
};
} // namespace GIMBAL