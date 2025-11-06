#include "Cmd.hpp"
#include "MsgImpl.hpp"
#include "StmLog.hpp"

Cmd::Cmd() : eventGroup_(xEventGroupCreate()), rcHandler_(&RC_UART, eventGroup_)
{
    msgBus_.chassisQueue = xQueueCreate(10, sizeof(ChassisMsg_s));
    msgBus_.gimbalQueue = xQueueCreate(10, sizeof(GimbalMsg_s));
    msgBus_.armorBoosterQueue = xQueueCreate(10, sizeof(ArmorBoosterMsg_s));
    msgBus_.armorLoaderQueue = xQueueCreate(10, sizeof(ArmorLoaderMsg_s));
    msgBus_.refereeQueue = xQueueCreate(10, sizeof(RefereeMsg_s));

    rttHandler_.init(&msgBus_, eventGroup_);
    rcHandler_.init(&msgBus_, eventGroup_);
#if defined APP_USE_REFEREE
    refereeHandler_.init(&msgBus_, eventGroup_);
#endif

    xTaskCreate(Cmd::task, "cmd_task", 256, this, 2, nullptr);

    LOG::info("cmd", "init success");
}


void Cmd::parseMsg()
{
    EventBits_t xBits = xEventGroupWaitBits(eventGroup_, EVENT_MASK, pdTRUE,
                                            pdFALSE, portMAX_DELAY);
    if (xBits & (RC_READY_EVENT | ET08A_READY_EVENT)) {
        rcHandler_.handle();
    }
    if (xBits & RTT_READY_EVENT) {
        rttHandler_.handle();
    }
#if defined APP_USE_REFEREE
    if (xBits & REFEREE_READY_EVENT) {
        refereeHandler_.handle();
    }
#endif
}

void Cmd::task(void *_param)
{
    auto instance = static_cast<Cmd *>(_param);
    for (;;) {
        instance->parseMsg();
    }
}
