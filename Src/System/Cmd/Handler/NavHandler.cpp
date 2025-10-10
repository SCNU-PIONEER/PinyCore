#include "NavHandler.hpp"
#include "Nav.hpp"
#include "sdkconfig.h"

void NavHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    this->msgBus_ = _bus;
    this->event = _event;
    nav = new NAV::Nav();
    nav->navInit(_event);
}

void NavHandler::handle()
{
    msg_.decision = nav->getNavToEcDataPtr()->decision;
    msg_.vx = nav->getNavToEcDataPtr()->vx;
    msg_.vy = nav->getNavToEcDataPtr()->vy;
    msg_.wz = nav->getNavToEcDataPtr()->wz;
    notify(&msg_, msgBus_->navigationQueue);
}

void NavHandler::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0);
}
