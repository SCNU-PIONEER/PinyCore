
#include "RefereeHandler.hpp"
#include "sdkconfig.h"

void RefereeHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    this->msgBus_ = _bus;
    this->event = _event;
    extern UART_HandleTypeDef REFEREE_UART;
    referee_.init(&REFEREE_UART, _event);
}

void RefereeHandler::handle()
{
    referee_.readRefereeData();

    msg_.bulletSpeed = referee_.getRefereeData().shootData.bulletSpeed;
    msg_.shooterHeatLimit =
            referee_.getRefereeData().gameRobotStatus.shooterHeatLimit;
    msg_.chassisPowerLimit =
            referee_.getRefereeData().gameRobotStatus.chassisPowerLimit;
    msg_.chassisPowerBuffer =
            referee_.getRefereeData().powerHeatData.chassisPowerBuffer;
    msg_.currentHP = referee_.getRefereeData().gameRobotStatus.currentHP;

    notify(&msg_, msgBus_->refereeQueue);
}

void RefereeHandler::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}