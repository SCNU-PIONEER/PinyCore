#include "Bsp_uart.hpp"
#include "Gimbal.hpp"
#include "MotorCommonMacros.hpp"
#include "DualStage.hpp"

#include "Nav.hpp"
#include "PidBasic.hpp"

#include "GM6020.hpp"
#include "DM4310.hpp"
#include "Bsp_Usb.hpp"
#include "Vision.hpp"
#include "sdkconfig.h"
#include "Bsp_can.hpp"

#include "./State/StopState.hpp"
#include "./State/LaunchState.hpp"
#include "./State/RunState.hpp"
#include "./State/PatrolState.hpp"
#include "./State/AutoState.hpp"
#include "./State/NavState.hpp"
#include <cstdint>


extern canHandle HCAN1;

using namespace GIMBAL;
using namespace PINYMOTOR;

static FILTER::LPF gimbalFilterOutput[3]{
    FILTER::LPF(1000.f, 4.f),
    FILTER::LPF(1000.f, 5.f),
    FILTER::LPF(1000.f, 4.f),
};

void ChassisCtrlTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN3, uid(), _buf, _len);
}

void ShooterCtrlTxPacket::transmit(uint8_t *_buf, uint16_t _len)
{
    Can::instance().transmitData(&HCAN3, uid(), _buf, _len);
}


DualStage::DualStage()
{
    for (uint8_t i = 0; i < 3; i++) {
        gimbalFilterOutput_._[i] = &gimbalFilterOutput[i];
    }

    isGimbalResverse.Pitch = DualStage::PITCH_REVERSE;
    isGimbalResverse.FirstYaw = DualStage::FIRST_YAW_REVERSE;
    isGimbalResverse.SecondYaw = DualStage::SECOND_YAW_REVERSE;

    InitConfig_s pitchDM4310Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 1,
        .txFreq = PITCH_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = PITCH_REVERSE,
    };
    motors_.Pitch = new DMMOTOR::DM4310("Pitch", pitchDM4310Config);
    motors_.Pitch->setZeroAng(PITCH_LAUNCH_ANG);
    LOG::info("Gimbal", "Pitch zeros has set to %f", motors_.Pitch->data().zeroAng);

    InitConfig_s firstYawDM4310Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN3),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::MIT_TT,
        .offsetId = 2,
        .txFreq = FIRST_YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = FIRST_YAW_REVERSE,
    };
    motors_.FirstYaw = new DMMOTOR::DM4310("Yaw", firstYawDM4310Config);
    motors_.FirstYaw->setZeroAng(FIRST_YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Big yaw zeros has set to %f", motors_.FirstYaw->data().zeroAng);

    InitConfig_s secondYawGM6020Config = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 3,
        .txFreq = SECOND_YAW_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = SECOND_YAW_REVERSE,
    };
    motors_.SecondYaw = new DJIMOTOR::GM6020("Yaw", secondYawGM6020Config);
    motors_.SecondYaw->setZeroAng(SECOND_YAW_LAUNCH_ANG);
    LOG::info("Gimbal", "Small yaw zeros has set to %f", motors_.SecondYaw->data().zeroAng);

    this->stateFactory_.addState(FSMState_e::RUN, std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::LAUNCH, std::make_unique<LaunchState>(this));
    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    this->stateFactory_.addState(FSMState_e::PATROL, std::make_unique<PatrolState>(this));
    this->stateFactory_.addState(FSMState_e::AUTO, std::make_unique<AutoState>(this));
    this->stateFactory_.addState(FSMState_e::NAV, std::make_unique<NavState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));
}

void DualStage::stop()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::OFF);
    }
}

void DualStage::enter()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::ON);
    }
}

void DualStage::update(void *_param)
{
    this->insSub_->receive();

    if (xQueueReceive((((MsgBus_s *)_param)->gimbalQueue), &msg, 0) == pdTRUE) {
    };

    for (uint8_t i = 0; i < 3; i++) {
        curMotorAng_._[i] = motors_._[i]->posNorm(); // 0~2*pi
    }
    //ins -pi~pi
    curImuAng_.Pitch = insData_.pitch;
    curImuAng_.SecondYaw = insData_.yaw; //云台imu在小yaw上
    curImuAng_.FirstYaw = getMinorArc(curImuAng_.SecondYaw, rangeConversion(curMotorAng_.SecondYaw));

    compensateImuAngle();

    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg, 0) == pdTRUE) {
    };
    chassisCtrlTxPacket_.setData().state = static_cast<uint8_t>(cmsg.state);

    if (xQueueReceive((((MsgBus_s *)_param)->navigationQueue), &nmsg, 0) == pdTRUE) {
    };
#if APP_USE_NAV
    if (msg.state == FSMState_e::NAV) {
        chassisCtrlTxPacket_.setData().vx = cmsg.vx + nmsg.vx;
        chassisCtrlTxPacket_.setData().vy = cmsg.vy + nmsg.vy;
        chassisCtrlTxPacket_.setData().wz = cmsg.wz + nmsg.wz;
    } else if (msg.state == FSMState_e::RUN) {
        chassisCtrlTxPacket_.setData().vx = cmsg.vx;
        chassisCtrlTxPacket_.setData().vy = cmsg.vy;
        chassisCtrlTxPacket_.setData().wz = cmsg.wz;
    }
#else
    chassisCtrlTxPacket_.setData().vx = cmsg.vx;
    chassisCtrlTxPacket_.setData().vy = cmsg.vy;
    chassisCtrlTxPacket_.setData().wz = cmsg.wz;
#endif
    chassisCtrlTxPacket_.setData().deltaYaw = motors_.FirstYaw->posNorm();

    ArmorLoaderMsg_s almsg{};
    if (xQueueReceive((((MsgBus_s *)_param)->armorLoaderQueue), &almsg, 0) == pdTRUE) {
        shooterCtrlTxPacket_.setData().state = static_cast<uint8_t>(almsg.state);
        shooterCtrlTxPacket_.setData().fire = almsg.fire;
    }
    shooterCtrlTxPacket_.setData().fire &= true; // TODO: vision fire

    // vision send
    vision->eToVData().roll = insData_.roll;
    vision->eToVData().pitch = insData_.pitch;
    vision->eToVData().yaw = insData_.yaw;
    vision->eToVData().timestamp = xTaskGetTickCount();

    //nav send
    nav->ecToNavData().pitch = insData_.pitch;
    nav->ecToNavData().yaw = motors_.FirstYaw->posNorm();

    this->stateFactory_.update();
}

void DualStage::compensateImuAngle()
{
    float tempComAng;
    if (this->isConfirmYawZero_) {
        //控制大yaw
        this->curImuAng_.SecondYaw = getMinorArc(this->curImuAng_.SecondYaw, -this->constDeltaYawArc_);
        this->curImuAng_.FirstYaw = getMinorArc(this->curImuAng_.FirstYaw, -this->constDeltaYawArc_);
        //与gimbal类保持相同
        this->ctrlYawAng_ = this->curImuAng_.FirstYaw;
    } else {
        // confirm zero
        if (motors_.FirstYaw->posNorm() != 0.f && motors_.SecondYaw->posNorm() != 0.f) {
            // TODO: prefer to globalState
            /*实际在获取上电后imu确定的坐标系与车体坐标系正方向的差角,当前imu在小yaw上.
            小yaw+大yaw=imu坐标系与车体坐标系正方向的差角*/
            tempComAng = motors_.SecondYaw->posNorm() + motors_.FirstYaw->posNorm();
            //0~2pi
            tempComAng = PINYMOTOR::rangeMap(tempComAng);
            this->constDeltaYawArc_ = FIRST_YAW_REVERSE ? getMinorArc(0.f, tempComAng) : getMinorArc(tempComAng, 0.f);
            this->isConfirmYawZero_ = true;
        } else {
            this->isConfirmYawZero_ = false;
        }
    }
}

void DualStage::updateBaseYaw() {}

void DualStage::ctrl(DualStage_u<float> _pos, DualStagePtr_u<PositionalPid> _angPID,
                     DualStagePtr_u<PositionalPid> _velPID)
{
    float del[3], vel[3], torq[3];
    for (uint8_t i = 0; i < 3; i++) {
        del[i] = getMinorArc(_pos._[i], motors_._[i]->posNorm());
        vel[i] = _angPID._[i]->calc(del[i], 0);
        torq[i] = _velPID._[i]->calc(vel[i], motors_._[i]->vel());
        if (msg.state == FSMState_e::STOP) {
            torq[i] = 0.f;
        }
        motors_._[i]->cmdTorq(gimbalFilterOutput_._[i]->process(torq[i]));
    }
}

void DualStage::ctrl(DualStage_u<float> _pos, DualStage_u<float> _vel, DualStagePtr_u<PositionalPid> _angPID,
                     DualStagePtr_u<PositionalPid> _velPID)
{
    float del[3], vel[3], torq[3];
    for (uint8_t i = 0; i < 3; i++) {
        del[i] = getMinorArc(_pos._[i], motors_._[i]->posNorm());
        vel[i] = _angPID._[i]->calc(fmax(-_vel._[i], fmin(del[i], _vel._[i])), 0);
        torq[i] = _velPID._[i]->calc(vel[i], motors_._[i]->vel());
        motors_._[i]->cmdTorq(gimbalFilterOutput_._[i]->process(torq[i]));
    }
}

void DualStage::loadResetState(bool _state) { chassisCtrlTxPacket_.setData().gimbalReset = _state; }
