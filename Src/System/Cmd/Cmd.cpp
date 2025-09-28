#include "Cmd.hpp"
#include "MsgImpl.hpp"
#include "StmLog.hpp"
#include "sdkconfig.h"

Cmd::Cmd() : eventGroup_(xEventGroupCreate())
{
    msgBus_.chassisQueue = xQueueCreate(30, sizeof(ChassisMsg_s));
    msgBus_.gimbalQueue = xQueueCreate(30, sizeof(GimbalMsg_s));
    msgBus_.armQueue = xQueueCreate(30, sizeof(ArmMsg_s));
    msgBus_.refereeQueue = xQueueCreate(30, sizeof(RefereeMsg_s));

    rttHandler_.init(&msgBus_, eventGroup_);
    rcHandler_.init(&msgBus_, eventGroup_);
    if constexpr (APP_USE_REFEREE)
        refereeHandler_.init(&msgBus_, eventGroup_);

    xTaskCreate(Cmd::task, "cmd_task", 256, this, 1, nullptr);

    LOG::info("cmd", "init success");
}


void Cmd::parseMsg()
{
    EventBits_t xBits = xEventGroupWaitBits(eventGroup_, EVENT_MASK, pdTRUE,
                                            pdFALSE, portMAX_DELAY);
    if (xBits & RC_READY_EVENT) {
        rcHandler_.handle();
    }
    if (xBits & RTT_READY_EVENT) {
        rttHandler_.handle();
    }
    if constexpr (APP_USE_REFEREE) {
        if (xBits & REFEREE_READY_EVENT) {
            refereeHandler_.handle();
        }
    }
}

void Cmd::task(void *_param)
{
    auto instance = static_cast<Cmd *>(_param);
    for (;;) {
        instance->parseMsg();
    }
}
