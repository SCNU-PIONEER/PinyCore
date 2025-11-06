
#include "RefereeHandler.hpp"
#include "sdkconfig.h"

void RefereeHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    this->msgBus_ = _bus;
    this->event = _event;
    refereeRx = std::make_unique<REFEREE::RefReceiver>(&REFEREE_UART);
    refereeTx = std::make_unique<REFEREE::RefTransmitter>(&REFEREE_UART);

    refereeRx->init(_event);
}

void RefereeHandler::handle()
{
    refereeRx->readRefereeData();
    refereeRx->rxFreqCalc();

    msg_.bulletSpeed = refereeRx->getRefereeData().shootData.bulletSpeed;
    msg_.shooterHeatLimit =
            refereeRx->getRefereeData().gameRobotStatus.shooterHeatLimit;
    msg_.chassisPowerLimit =
            refereeRx->getRefereeData().gameRobotStatus.chassisPowerLimit;
    msg_.chassisPowerBuffer =
            refereeRx->getRefereeData().powerHeatData.chassisPowerBuffer;
    msg_.currentHP = refereeRx->getRefereeData().gameRobotStatus.currentHP;
    msg_.rxFreq = refereeRx->getRxFreq();

    notify(&msg_, msgBus_->refereeQueue);
}

void RefereeHandler::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}
