#include "MiniPcHandler.hpp"
#include "sdkconfig.h"
#include <memory>

void MiniPcHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    this->msgBus_ = _bus;
    this->event = _event;
#if APP_USE_VISION
    vision = std::make_unique<MINIPC::Vision>();
    vision->init(_event);
#endif
#if APP_USE_NAV
    nav = std::make_unique<MINIPC::Nav>();
    nav->init(_event);
#endif
}

void MiniPcHandler::handle()
{
#if APP_USE_VISION
    vision->parseData();
    vision->rxFreqCalc();
    visionMsg_.x = vision->getVisionToEcDataPtr().x;
    visionMsg_.y = vision->getVisionToEcDataPtr().y;
    visionMsg_.z = vision->getVisionToEcDataPtr().z;
    notify(&visionMsg_, msgBus_->visionQueue);
#endif
#if APP_USE_NAV
    nav->parseData();
    nav->rxFreqCalc();
    navMsg_.decision = nav->getNavToEcDataPtr().decision;
    navMsg_.vx = nav->getNavToEcDataPtr().vx;
    navMsg_.vy = nav->getNavToEcDataPtr().vy;
    navMsg_.wz = nav->getNavToEcDataPtr().wz;
    notify(&navMsg_, msgBus_->navigationQueue);
#endif
}

void MiniPcHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }
