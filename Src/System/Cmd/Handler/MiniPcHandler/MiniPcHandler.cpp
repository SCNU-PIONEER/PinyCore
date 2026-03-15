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
    visionMsg_.control = vision->v2eData().control;
    visionMsg_.fire = vision->v2eData().fire;
    visionMsg_.pitch = vision->v2eData().pitch;
    visionMsg_.pitchAcc = vision->v2eData().pitchAcc;
    visionMsg_.pitchVel = vision->v2eData().pitchVel;
    visionMsg_.yaw = vision->v2eData().yaw;
    visionMsg_.yawAcc = vision->v2eData().yawAcc;
    visionMsg_.yawVel = vision->v2eData().yawVel;
    notify(&visionMsg_, msgBus_->visionQueue);
}
void VisionHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }