
#include "RefereeHandler.hpp"
#include "sdkconfig.h"

void RefereeHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    this->msgBus_ = _bus;
    this->event = _event;

    referee = std::make_unique<REFEREE::Referee>(&REFEREE_UART, _event, this->bit_);
}

void RefereeHandler::handle()
{
    auto &rx = referee->receiver;

    rx.readRefereeData();
    rx.rxFreqCalc();

    msg_.bulletSpeed = rx.getRefereeData().shootData.bulletSpeed;
    msg_.shooterHeatLimit = rx.getRefereeData().gameRobotStatus.shooterHeatLimit;
    msg_.chassisPowerLimit = rx.getRefereeData().gameRobotStatus.chassisPowerLimit;
    msg_.chassisPowerBuffer = rx.getRefereeData().powerHeatData.chassisPowerBuffer;
    msg_.currentHP = rx.getRefereeData().gameRobotStatus.currentHP;
    msg_.rxFreq = rx.getRxFreq();

    notify(&msg_, msgBus_->refereeQueue);
}

void RefereeHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }
