/*
 * @Author: IRON.NY 2537933690@qq.com
 * @Date: 2025-12-12 16:07:55
 * @LastEditors: IRON.NY 2537933690@qq.com
 * @LastEditTime: 2026-03-26 23:49:10
 * @FilePath: \PinyCore\Src\App\RobotModules\Gimbal\Actuators\Standard\Standard.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <stdio.h>
#include "Leather.hpp"

#include "M3508.hpp"
#include "GM6020.hpp"

#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"

#include "Bsp_pwm.hpp"

uint8_t constdartId = 0;

using namespace SHOOTER;
using namespace PINYMOTOR;

Leather::Leather()
{
    InitConfig_s shooterLM3508Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = 1000.0f,
        .isReverse = false,
    };

    InitConfig_s shooterRM3508Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 2,
        .txFreq = 1000.0f,
        .isReverse = false,
    };
    InitConfig_s shooterTurnWheelConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 3,
        .txFreq = 1000.0f,
        .isReverse = false,
    };

    motors_.shooterL = new DJIMOTOR::M3508("GM3508", shooterLM3508Config);
    motors_.shooterR = new DJIMOTOR::M3508("GM3508", shooterRM3508Config);
    motors_.shooterT = new DJIMOTOR::GM6020("GM6020", shooterTurnWheelConfig);

    ShooterLMotorCtrl.posDualPid = new PositionalPid(50.f, 0.01f, 0.f, 0.1f / 1000.f, 5.f, 16384.f / 10, 0.f);
    ShooterLMotorCtrl.velDualPid = new PositionalPid(0.0018f, 0.f, 0.f, 1.f / 1000.f, 1.5f, 16384.f, 0.f);
    ShooterLMotorCtrl.velSingPid = new PositionalPid(0.0125f, 0.f, 0.00001f, 1.f / 1000.f, 1.5f, 16384.f, 0.f);
    
    ShooterRMotorCtrl.posDualPid = new PositionalPid(50.f, 0.01f, 0.f, 0.1f / 1000.f, 5.f, 16384.f / 10, 0.f);
    ShooterRMotorCtrl.velDualPid = new PositionalPid(0.0018f, 0.f, 0.f, 1.f / 1000.f, 1.5f, 16384.f, 0.f);
    ShooterRMotorCtrl.velSingPid = new PositionalPid(0.0125f, 0.f, 0.00001f, 1.f / 1000.f, 1.5f, 16384.f, 0.f);
    
    ShooterTMotorCtrl.posDualPid = new PositionalPid(7.25f, 0.01f, 0.006f, 0.1f / 1000.f, 5.f, 16384.f / 10, 0.f);
    ShooterTMotorCtrl.velDualPid = new PositionalPid(2.5f, 0.f, 0.025f, 1.f / 1000.f, 1.5f, 16384.f, 0.f);

    stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    stateFactory_.addState(FSMState_e::LAUNCH, std::make_unique<LaunchState>(this));
    stateFactory_.addState(FSMState_e::RUN, std::make_unique<RunState>(this));
    stateFactory_.init(stateFactory_.getNextState(FSMState_e::STOP));
}

void Leather::stop()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::OFF);
    }

}

void Leather::enter()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::ON);
    }
}

volatile float angTurnwheelDisp;
volatile float angTurnwheelExp;
volatile float angScrewDisp;
volatile float angScrewExp;
volatile float torqScrew;
void Leather::updateSelf()
{
//---------------------------Ozone Debug----------------------------------------------------------------------
    angTurnwheelDisp = this->shooterT()->data().ang;
    angTurnwheelExp = this->cmd_turnWheelPos;
    angScrewDisp = this->screwState().pos;
    angScrewExp = this->cmd_screwPos;
    torqScrew = (fabsf(this->shooterL()->data().torq) + fabsf(this->shooterR()->data().torq)) / 2.f;
//------------------------------------------------------------------------------------------------------------    
//----------------------------------Dual board contact---------------------------------------------------------
    uint8_t dualBoardstr[8] = {0};
    sprintf((char*)dualBoardstr, "/%d,%d,%d/", constdartId, this->magFlag[constdartId], this->servoFlag[constdartId]);
    Can::instance().transmitData(&HCAN2, 0x123, dualBoardstr, strlen((char*)dualBoardstr));
//-------------------------------------------------------------------------------------------------------------  
    this->screwState_.pos = (this->shooterL()->data().cirNum + this->shooterR()->data().cirNum) / 2.f 
                               - screw_zeroCirNum;
    this->turnwheelState_.pos= this->shooterT()->data().ang;
    this->stateFactory_.update();
}

void Leather::shooterServoCtrl(){
    if(this->fireFlag)
        pwm_->setDutyCycle(1500);
    else
        pwm_->setDutyCycle(500);
}

void Leather::getsZeroPos(){
    screw_zeroCirNum = (motors_.shooterL->data().cirNum + motors_.shooterR->data().cirNum) / 2.f;
}

void Leather::turnWheelCtrl(){
    // float targetPos = 0.f;
    // targetPos = std::clamp(cmd_turnWheelPos, turnwheelState().pos - 0.2f, turnwheelState().pos + 0.2f);
    float deltaAng = PINYMOTOR::getMinorArc(cmd_turnWheelPos, //弧度制
                                                shooterT()->data().ang);
    if(fabsf(deltaAng) > 0.2f)
        deltaAng = 0.2f * (deltaAng > 0 ? 1.0f : -1.0f);
    else{;}
    ShooterTMotorCtrl.targetVel = ShooterTMotorCtrl.posDualPid->calc(deltaAng, 0.f);

    ShooterTMotorCtrl.torq = torqLpf[0].process(
                                ShooterTMotorCtrl.velDualPid->calc(ShooterTMotorCtrl.targetVel, 
                                                                    shooterT()->data().spdRadps));

    motors_.shooterT->cmdTorq(ShooterTMotorCtrl.torq);
}

volatile float rpmShooterLDisp;
volatile float rpmShooterRDisp;
volatile float rpmShooterExp;
volatile float torqShooterL;
void Leather::screwSingRpmCtrl(float expsRpm){
    float _LcurRpm = motors_.shooterL->vel() / (2 * PI);
    float _RcurRpm = motors_.shooterR->vel() / (2 * PI);
//-------------------ozone debug--------------------
    rpmShooterLDisp = _LcurRpm;
    rpmShooterRDisp = _RcurRpm;
    rpmShooterExp = expsRpm;
    torqShooterL = this->motors_.shooterL->data().torq;
//--------------------------------------------------
    ShooterLMotorCtrl.targetVel = expsRpm;
    ShooterRMotorCtrl.targetVel = expsRpm;

    // ShooterLMotorCtrl.torq = torqLpf[0].process(
    //                             ShooterLMotorCtrl.velSingPid->calc(ShooterLMotorCtrl.targetVel, 
    //                                                                 _LcurRpm));
    ShooterLMotorCtrl.torq = ShooterLMotorCtrl.velSingPid->calc(ShooterLMotorCtrl.targetVel, 
                                                                    _LcurRpm);                                                            
    motors_.shooterL->cmdTorq(ShooterLMotorCtrl.torq);
    
    // ShooterRMotorCtrl.torq = torqLpf[0].process(
    //                         ShooterRMotorCtrl.velSingPid->calc(ShooterRMotorCtrl.targetVel, 
    //                                                             _RcurRpm));
    ShooterRMotorCtrl.torq = ShooterRMotorCtrl.velSingPid->calc(ShooterRMotorCtrl.targetVel, 
                                                                    _RcurRpm);   
    motors_.shooterR->cmdTorq(ShooterRMotorCtrl.torq);
}

void Leather::screwDualAngCtrl(){
    float targetPos_load = std::clamp(cmd_screwPos, screwState().pos - loopAcc, screwState().pos + loopAcc);
    float deltaAng = targetPos_load - screwState().pos;
    float targetLvel_load = ShooterLMotorCtrl.posDualPid->calc(deltaAng, 0.f);
    float targetRvel_load = ShooterRMotorCtrl.posDualPid->calc(deltaAng, 0.f);
    
    ShooterLMotorCtrl.targetVel = targetLvel_load;//无减速箱
    ShooterRMotorCtrl.targetVel = targetRvel_load;//无减速箱

    ShooterLMotorCtrl.torq = ShooterLMotorCtrl.velDualPid->calc(ShooterLMotorCtrl.targetVel, this->motors_.shooterL->data().spdRadps);
    ShooterRMotorCtrl.torq = ShooterRMotorCtrl.velDualPid->calc(ShooterRMotorCtrl.targetVel, this->motors_.shooterR->data().spdRadps); 
    shooterL()->cmdTorq(ShooterLMotorCtrl.torq);
    shooterR()->cmdTorq(ShooterRMotorCtrl.torq);
}