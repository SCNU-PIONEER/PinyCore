/*
 * @Author: IRON.NY 2537933690@qq.com
 * @Date: 2025-12-12 16:07:55
 * @LastEditors: IRON.NY 2537933690@qq.com
 * @LastEditTime: 2026-03-25 20:28:24
 * @FilePath: \PinyCore\Src\App\RobotModules\Gimbal\Actuators\Standard\Standard.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Standard.hpp"

#include "GM6020.hpp"
#include "M2006.hpp"

#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"

#include "Bsp_usb.hpp"

bool constloadedGimbal = false;
extern canHandle HCAN1;

using namespace GIMBAL;
using namespace PINYMOTOR;

Standard::Standard()
{
    InitConfig_s yawGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = 1000.0f,
        .isReverse = YAW_REVERSE,
    };
    InitConfig_s trigM2006Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 3,
        .txFreq = 1000.0f,
        .isReverse = false,
    };

    motors_.yaw = new DJIMOTOR::GM6020("GM6020", yawGM6020Config);
    motors_.trig = new DJIMOTOR::M2006("M2006", trigM2006Config);

  //  motors_.yaw->setZeroAng(YAW_LAUNCH_ANG);

    YawMotorCtrl.posDualPid = new PositionalPid(18.f, 0.01f, 0.f, 0.1f / 1000.f, 5.f, 16384.f / 10, 0.f);
    YawMotorCtrl.velDualPid = new PositionalPid(0.1f, 0.f, 0.0023f, 1.f / 1000.f, 1.5f, 16384.f, 0.f);
    YawMotorCtrl.velSingPid = new PositionalPid(0.1f, 0.f, 0.001f, 1.f / 1000.f, 1.5f, 16384.f, 0.f);

    TrigMotorCtrl.posDualPid = new PositionalPid(15.f, 0.01f, 0.f, 0.1f / 1000.f, 5.f, 10000.f / 10, 0.f);
    TrigMotorCtrl.velDualPid = new PositionalPid(0.00035f, 0.f, 0.000001f, 1.f / 1000.f, 1.5f, 10000.f, 0.f);
    TrigMotorCtrl.velSingPid = new PositionalPid(0.00075f, 0.f, 0.f, 1.f / 1000.f, 1.5f, 10000.f, 0.f);

    stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    stateFactory_.addState(FSMState_e::LAUNCH, std::make_unique<LaunchState>(this));
    stateFactory_.addState(FSMState_e::RUN, std::make_unique<RunState>(this));
    stateFactory_.init(stateFactory_.getNextState(FSMState_e::STOP));
}

void Standard::stop()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::OFF);
    }

    cmd_yawPos = yawState_.pos;
}

void Standard::enter()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::ON);
    }
}

volatile float angYawDisp;
volatile float angYawExp;
volatile float rpmTrigDisp;
volatile float rpmTrigExp;
volatile float angTrigDisp;
volatile float angTrigExp;
void Standard::updateSelf()
{
//---------------------------Ozone Debug----------------------------------------------------------------------
    angYawDisp = this->yawState().pos;
    angYawExp = this->cmd_yawPos;
    
    angTrigDisp = this->trigState().pos;
    angTrigExp = this->cmd_trigPos;

    rpmTrigDisp = this->motors_.trig->data().spdRpm;
    rpmTrigExp = this->gmsg.gainTrig;
//-------------------------------------------------------------------------------------------------------------       
    
    
//yaw负载
    this->yawState_.pos = ((motors_.yaw->data().multipCirAng - yaw_zeroAng) / (2 * PI) /
                            (ratioAngle2Loop));//角度制
    this->yawState_.vel = (motors_.yaw->data().spdRadps /
                            (ratioAngle2Loop));
//trig负载
    this->trigState_.pos = (motors_.trig->data().cirNum - trig_zeroCirNum) / ratio2006Gear;//圈数
    this->trigState_.vel = motors_.trig->data().spdRadps / ratio2006Gear;  
    
    vision_.setData().yaw = this->yawState().pos;
    this->stateFactory_.update();

}

void Standard::yawDualAngCtrl()
{
    float targetPos = std::clamp(cmd_yawPos, yawState().pos - angAcc, yawState().pos + angAcc);

    float deltaAng = targetPos - yawState().pos;//0~10度，无需劣弧
    YawMotorCtrl.targetVel = YawMotorCtrl.posDualPid->calc(deltaAng, 0.f);
    YawMotorCtrl.torq = torqLpf[0].process(YawMotorCtrl.velDualPid->calc(YawMotorCtrl.targetVel, this->yawState().vel));
    //YawMotorCtrl.torq = YawMotorCtrl.velDualPid->calc(YawMotorCtrl.targetVel, this->yawState().vel);
    yaw()->cmdTorq(YawMotorCtrl.torq);
}

void Standard::yawSingRpmCtrl(float _expRpm)
{
    float _curRpm = motors_.yaw->vel() / (2 * PI);
    YawMotorCtrl.targetVel = std::clamp(_expRpm, _curRpm - 10.f, _curRpm + 10.f);
    YawMotorCtrl.torq = torqLpf[0].process(YawMotorCtrl.velSingPid->calc(YawMotorCtrl.targetVel, _curRpm));
    yaw()->cmdTorq(YawMotorCtrl.torq);
}

void Standard::getyZeroPos(){
    yaw_zeroAng = motors_.yaw->data().multipCirAng;
    trig_zeroCirNum = motors_.trig->data().cirNum;
}

void Standard::gettZeroPos(){
    trig_zeroCirNum = motors_.trig->data().cirNum;
}

void Standard:: trigDualAngCtrl(){
    float targetPos_load = std::clamp(cmd_trigPos, trigState().pos - loopAcc, trigState().pos + loopAcc);
    float deltaAng = targetPos_load - trigState().pos;//0~10度，无需劣弧
    float targetvel_load = std::clamp(TrigMotorCtrl.posDualPid->calc(deltaAng, 0.f), -20.f, 20.f);

    TrigMotorCtrl.targetVel = targetvel_load * ratio2006Gear;
//    TrigMotorCtrl.torq = torqLpf[1].process(TrigMotorCtrl.velDualPid->calc(TrigMotorCtrl.targetVel, this->trigState().vel * ratioGear));
    TrigMotorCtrl.torq = (TrigMotorCtrl.velDualPid->calc(TrigMotorCtrl.targetVel, this->trigState().vel * ratio2006Gear));
    trig()->cmdTorq(TrigMotorCtrl.torq);
}

void Standard:: trigSingRpmCtrl(float _expRpm){
    float _curRpm = motors_.trig->vel() / (2 * PI) * 60.f;
    TrigMotorCtrl.targetVel = std::clamp(_expRpm, _curRpm - 10.f, _curRpm + 10.f);
    TrigMotorCtrl.torq = torqLpf[1].process(TrigMotorCtrl.velSingPid->calc(TrigMotorCtrl.targetVel, _curRpm));
    //TrigMotorCtrl.torq = TrigMotorCtrl.velSingPid->calc(TrigMotorCtrl.targetVel, _curRpm);
    trig()->cmdTorq(TrigMotorCtrl.torq);
}
