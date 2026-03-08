
#include "RefereeHandler.hpp"
#include "sdkconfig.h"

#if APP_USE_UI
#include "UI/UIApp.hpp"
#endif

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

    msg_.robotId = rx.getRefereeData().gameRobotStatus.robotId;

#if APP_USE_UI
    UI::APP::instance()->client().updateID(msg_.robotId);
#endif

    msg_.remainHeat = rx.getRefereeData().gameRobotStatus.shooterHeatLimit -
                      ((msg_.robotId % 100 == 1) ? rx.getRefereeData().powerHeatData.shooter_42mmBarrelHeat :
                                                   rx.getRefereeData().powerHeatData.shooter_17mmBarrelHeat);
    msg_.chassisPowerLimit = rx.getRefereeData().gameRobotStatus.chassisPowerLimit;
    msg_.chassisPowerBuffer = rx.getRefereeData().powerHeatData.chassisPowerBuffer;
    msg_.bulletSpeed = rx.getRefereeData().shootData.bulletSpeed;
    msg_.currentHP = rx.getRefereeData().gameRobotStatus.currentHP;
    msg_.gameTime = rx.getRefereeData().gameStatus.stageRemainTime;
    msg_.isPlay = rx.getRefereeData().gameStatus.gameType == 0x01;
    msg_.rxFreq = rx.getRxFreq();

    notify(&msg_, msgBus_->refereeQueue);
}

void RefereeHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }
