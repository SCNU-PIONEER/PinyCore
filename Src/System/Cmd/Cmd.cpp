#include "Cmd.hpp"
#include "MsgImpl.hpp"
#include "StmLog.hpp"

Cmd::Cmd()
        : Task<Cmd, 256>("CmdTask", TaskPriority_e::LOW2)
        , eventGroup_(xEventGroupCreate())
        , rcHandler_(&RC_UART, eventGroup_)

{
    msgBus_.chassisQueue = xQueueCreate(10, sizeof(ChassisMsg_s));
    msgBus_.gimbalQueue = xQueueCreate(10, sizeof(GimbalMsg_s));
    msgBus_.armorBoosterQueue = xQueueCreate(10, sizeof(ArmorBoosterMsg_s));
    msgBus_.armorLoaderQueue = xQueueCreate(10, sizeof(ArmorLoaderMsg_s));
    msgBus_.refereeQueue = xQueueCreate(10, sizeof(RefereeMsg_s));

    for (auto i : Handler::getHandlerList()) {
        i.handler->init(&msgBus_, eventGroup_);
    }

    LOG::info("cmd", "init success");
}


void Cmd::parseMsg()
{
    EventBits_t xBits =
            xEventGroupWaitBits(eventGroup_, Handler::getMasks().to_ulong(), pdTRUE, pdFALSE, portMAX_DELAY);
    for (auto i : Handler::getHandlerList()) {
        if (xBits & i.bit) {
            i.handler->handle();
        }
    }
}

void Cmd::task()
{
    for (;;) {
        parseMsg();
    }
}
