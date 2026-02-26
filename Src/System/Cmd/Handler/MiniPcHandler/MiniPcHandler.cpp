#include "MiniPcHandler.hpp"
#include "Vision.hpp"
#include "Nav.hpp"
#include "sdkconfig.h"
#include <memory>
#include "MiniPcManager.hpp"

using namespace MINIPC;

void NavHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    this->msgBus_ = _bus;
    this->event = _event;
    nav = std::make_unique<Nav>(_event, this->bit_);
};

void NavHandler::handle()
{
    nav->parseData();
    nav->rxFreqCalc();
    navMsg_.posture = nav->navToEData().posture;
    navMsg_.vx = nav->navToEData().vx;
    navMsg_.vy = nav->navToEData().vy;
    navMsg_.wz = nav->navToEData().wz;
    notify(&navMsg_, msgBus_->navigationQueue);
}

void NavHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }

void VisionHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    this->msgBus_ = _bus;
    this->event = _event;
    vision = std::make_unique<Vision>(_event, this->bit_);
}
void VisionHandler::handle()
{
    vision->parseData();
    vision->rxFreqCalc();
    notify(&visionMsg_, msgBus_->visionQueue);
}
void VisionHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }