#include "Chassis.hpp"
#include "ChassisRunState.hpp"
#include "ChassisStopState.hpp"
#include "ChassisFollowState.hpp"
#include "ChassisSpinState.hpp"
#include "ChassisLaunchState.hpp"

#include <memory>

using namespace CHASSIS;

Chassis::Chassis(Locomotion *_wheel) : deltaYaw(0.f), wheel(_wheel)
{
    stateFactory.addState(static_cast<uint8_t>(FSMState_e::STOP),
                          std::make_unique<StopState>(this));

    stateFactory.addState(static_cast<uint8_t>(FSMState_e::LAUNCH),
                          std::make_unique<LaunchState>(this));

    stateFactory.addState(static_cast<uint8_t>(FSMState_e::RUN),
                          std::make_unique<RunState>(this));

    stateFactory.addState(static_cast<uint8_t>(FSMState_e::FOLLOW),
                          std::make_unique<FollowState>(this));

    stateFactory.addState(static_cast<uint8_t>(FSMState_e::SPIN),
                          std::make_unique<SpinState>(this));

    stateFactory.init(
            stateFactory.getNextState(static_cast<uint8_t>(FSMState_e::STOP)));
}

void Chassis::update(void *_param)
{

    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &msg, 0) ==
        pdTRUE) {
    };
    stateFactory.update();


    deltaYaw = gmsg.yaw; //需要进一步处理
}

/**
 * @brief Convert the chassis coordinate system
 * @note  云台系速度转换为底盘系速度
 * @param: void
 * @return: 换系后速度
 */
void Chassis::convertCoordinate()
{
    float tempVx, tempVy, tempWz;
    float sinYaw = sin(deltaYaw);
    float cosYaw = cos(deltaYaw);

    tempVx = -sinYaw * msg.vy + cosYaw * msg.vx;
    tempVy = cosYaw * msg.vy + sinYaw * msg.vx;

    msg.vx = tempVx;
    msg.vy = tempVy;
}
