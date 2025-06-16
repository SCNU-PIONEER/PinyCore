#include "rttMsgHandler.hpp"
#include "SEGGER_RTT.h"
#include <string.h>

void RTTMsgHandler::init(EventGroupHandle_t _event)
{
    event = _event;
    TimerHandle_t xTimer = xTimerCreate("rttTime",         // 定时器名称
                                        pdMS_TO_TICKS(10), // 周期
                                        pdTRUE,            // 自动重载
                                        this,              // 定时器ID
                                        parse              // 回调函数
    );

    if (xTimer != NULL) {
        xTimerStart(xTimer, 0); // 第二个参数是阻塞时间(ticks)
    }
}


void RTTMsgHandler::parse(TimerHandle_t xTimer)
{
    if (SEGGER_RTT_HasKey()) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        RTTMsgHandler *handler =
                static_cast<RTTMsgHandler *>(pvTimerGetTimerID(xTimer));
        xEventGroupSetBitsFromISR(handler->event, RTT_READY_EVENT,
                                  &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void RTTMsgHandler::handle()
{
    memset(data, 0, sizeof(data));
    SEGGER_RTT_Read(0, data, sizeof(data) - 1);

    if (strcmp((const char *)data, "run\n") == 0) {
        cmsg.state = State_e::run;
    } else if (strcmp((const char *)data, "stop\n") == 0) {
        cmsg.state = State_e::stop;
    } else {
        cmsg.state = State_e::stop;
    }
}
