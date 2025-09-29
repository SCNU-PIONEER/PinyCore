
#include "RefereeHandler.hpp"
#include "sdkconfig.h"

void RefereeHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    this->msgBus_ = _bus;
    this->event = _event;
    referee = new REFEREE::RefReceiver(&REFEREE_UART);
    referee->init(_event);
}

void RefereeHandler::handle()
{
    referee->readRefereeData();

    msg_.bulletSpeed = referee->getRefereeData().shootData.bulletSpeed;
    msg_.shooterHeatLimit =
            referee->getRefereeData().gameRobotStatus.shooterHeatLimit;
    msg_.chassisPowerLimit =
            referee->getRefereeData().gameRobotStatus.chassisPowerLimit;
    msg_.chassisPowerBuffer =
            referee->getRefereeData().powerHeatData.chassisPowerBuffer;
    msg_.currentHP = referee->getRefereeData().gameRobotStatus.currentHP;

    notify(&msg_, msgBus_->refereeQueue);
}

void RefereeHandler::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}
