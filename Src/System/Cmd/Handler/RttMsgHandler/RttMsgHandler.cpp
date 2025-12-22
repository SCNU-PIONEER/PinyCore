#include "RttMsgHandler.hpp"
#ifdef CHASSIS_TYPE
#include CHASSIS_FILE
#endif
#include <cstring>
#include "SEGGER_RTT.h"


void RTTMsgHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
    TimerHandle_t xTimer = xTimerCreate("rttTime",         // 定时器名称
                                        pdMS_TO_TICKS(10), // 周期
                                        pdTRUE,            // 自动重载
                                        this,              // 定时器ID
                                        parse              // 回调函数
    );

    if (xTimer != nullptr) {
        xTimerStart(xTimer, 0); // 第二个参数是阻塞时间(ticks)
    }
}


void RTTMsgHandler::parse(TimerHandle_t _xTimer)
{
    if (SEGGER_RTT_HasKey()) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        RTTMsgHandler *handler = static_cast<RTTMsgHandler *>(pvTimerGetTimerID(_xTimer));
        xEventGroupSetBitsFromISR(handler->event, handler->bit_, &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

void RTTMsgHandler::handle()
{
    memset(data_, 0, sizeof(data_));
    SEGGER_RTT_Read(0, data_, sizeof(data_) - 1);
}

void RTTMsgHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }
