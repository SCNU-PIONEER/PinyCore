#include "Omni.hpp"
#include "Bsp_can.hpp"
#include "PidBasic.hpp"
#include <numbers>

#include "M3508.hpp"

#include "QuadricycleController.hpp"

#include "./State/StopState.hpp"
#include "./State/RunState.hpp"

extern canHandle HCAN1;
extern canHandle HCAN2;

using namespace CHASSIS;
using namespace PINYMOTOR;

constexpr float F_PI = std::numbers::pi_v<float>;
constexpr float SQRT_2 = std::numbers::sqrt2_v<float>;

void ChassisCtrlRxPacket::registerCallback()
{
    Can::instance().registerCallback(
            &HCAN1, uid(), [this](const uint8_t *_rxBuf) {
                xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
            });
    LOG::info("ChassisCtrlRxPacket", " Receive cb registered, uId:0x%hx \n",
              uid());
}

Omni::Omni()
{
#if !APP_USE_COMM
    // NOLINTBEGIN
    gimbalYawSub_ = new Subscriber<GimbalYawMsg_s>(
            1, &TopicRouter::instance().gimbalYawTopic, &gimbalYawMsg_);
    // NOLINTEND
#endif

    for (uint8_t i = 1; i <= 4; i++) {
        InitConfig_s m3508Config = {
            .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
            .comType = ComType_e::CAN,
            .workMode = WorkMode_e::QUAD_CURR,
            .offsetId = i,
            .txFreq = 100.0f,
            .posPID = nullptr,
            .velPID = new IncrementalPid(0.6f, 0.0035f, 0.f,
                                         DJIMOTOR::M3508::TORQ_MAX, 0.f),
            .torqPID = nullptr
        };
        motors_._[i - 1] = new DJIMOTOR::M3508("M3508", m3508Config);
    }

    this->stateFactory_.addState(FSMState_e::RUN,
                                 std::make_unique<RunState>(this));
    this->stateFactory_.addState(FSMState_e::STOP,
                                 std::make_unique<StopState>(this));
    this->stateFactory_.init(
            this->stateFactory_.getNextState(FSMState_e::STOP));

    if constexpr (APP_USE_POWERCTRL) {
        // TODO:
    }
}

void Omni::stop()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::OFF);
    }
}

void Omni::enter()
{
    for (auto &i : motors_._) {
        i->cmd(PINYMOTOR::MotorCmdType_e::ON);
    }
}

void Omni::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) ==
        pdTRUE) {
    };

    // TODO: yaw
    loadDeltaYaw();

    // forward
    for (uint8_t i = 0; i < 4; i++) {
        wSpeed_._[i] = velFltr_[i].process(motors_._[i]->data().spdRpm);
    }
    Speed_u v = forward(wSpeed_);
    curSpeed_.vx = v.vx * cosYaw_ + v.vy * sinYaw_;
    curSpeed_.vy = v.vy * cosYaw_ - v.vx * sinYaw_;

    // update state
    this->stateFactory_.update();
}

// rpm -> m/s
Speed_u Omni::forward(const WheelsSpeed_u &_wSpeed)
{
    Speed_u speed;
    speed.vx = W_CIRCUMFERENCE *
               (_wSpeed.M_LF - _wSpeed.M_RF + _wSpeed.M_LB - _wSpeed.M_RB) *
               (SQRT_2 / 4.f / 60.f);
    speed.vy = W_CIRCUMFERENCE *
               (-_wSpeed.M_LF - _wSpeed.M_RF + _wSpeed.M_LB + _wSpeed.M_RB) *
               (SQRT_2 / 4.f / 60.f);
    speed.wz = (W_DIAMETER / 2.f) *
               (_wSpeed.M_RF + _wSpeed.M_LF + _wSpeed.M_LB + _wSpeed.M_RB) *
               (1.f / (4.f / CHASSIS_R * 30.f / F_PI));
    return speed;
}

// m/s -> rad/s
WheelsSpeed_u Omni::reverse(const Speed_u &_speed)
{
    WheelsSpeed_u refWheels;
    refWheels.M_RF = (1.f / W_RADIUS) * (-_speed.vx - _speed.vy) / SQRT_2 +
                     CHASSIS_R / (W_DIAMETER / 2.f) * _speed.wz;
    refWheels.M_LF = (1.f / W_RADIUS) * (_speed.vx - _speed.vy) / SQRT_2 +
                     CHASSIS_R / (W_DIAMETER / 2.f) * _speed.wz;
    refWheels.M_LB = (1.f / W_RADIUS) * (_speed.vx + _speed.vy) / SQRT_2 +
                     CHASSIS_R / (W_DIAMETER / 2.f) * _speed.wz;
    refWheels.M_RB = (1.f / W_RADIUS) * (-_speed.vx + _speed.vy) / SQRT_2 +
                     CHASSIS_R / (W_DIAMETER / 2.f) * _speed.wz;
    return refWheels;
}

void Omni::ctrl(const Speed_u &_refSpeed)
{
    float vx = std::fmax(-MAX_VX_SPEED, std::fmin(_refSpeed.vx, MAX_VX_SPEED));
    float vy = std::fmax(-MAX_VY_SPEED, std::fmin(_refSpeed.vy, MAX_VY_SPEED));
    float wz = std::fmax(-MAX_WZ_SPEED, std::fmin(_refSpeed.wz, MAX_WZ_SPEED));
    Speed_u refSpeed = { .vx = (vx * this->cosYaw_) - (vy * this->sinYaw_),
                         .vy = (vy * this->cosYaw_) + (vx * this->sinYaw_),
                         .wz = wz };
    WheelsSpeed_u refWSpeed = reverse(refSpeed);

    for (uint8_t i = 0; i < 4; i++) {
        motors_._[i]->cmdVel(refWSpeed._[i]);
    }

    if constexpr (APP_USE_POWERCTRL) {
        // TODO:
    }
}

void Omni::loadDeltaYaw()
{
// Master Board method
#if !APP_USE_COMM
    if (gimbalYawSub_->receive()) {
        deltaYaw =
                getMinorArc(gimbalYawMsg_.followYaw, gimbalYawMsg_.chassisYaw);
    }

#endif
#ifdef APP_CHASSIS_SLAVE
    deltaYaw = getMinorArc(chassisCtrlRxPacket_.data().gimbalYaw, 0.f);
#endif
    cosYaw_ = arm_cos_f32(deltaYaw);
    sinYaw_ = arm_sin_f32(deltaYaw);
};

bool Omni::isGimbalReset() const
{
#if !APP_USE_COMM
    return gimbalYawMsg_.isReset;
#endif
#ifdef APP_CHASSIS_SLAVE
    return chassisCtrlRxPacket_.data().gimbalReset;
#endif
}
