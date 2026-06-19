#include "Cmd.hpp"
#include "MsgImpl.hpp"
#include "StmLog.hpp"

Cmd::Cmd() : Task<Cmd, 256>("CmdTask", TaskPriority_e::LOW2)

{
#if APP_USE_CHASSIS
    msgBus_.chassisQueue = xQueueCreate(30, sizeof(ChassisMsg_s));
#endif
#if APP_USE_GIMBAL
    msgBus_.gimbalQueue = xQueueCreate(30, sizeof(GimbalMsg_s));
#endif
#if EXTENSION_REFEREE
    msgBus_.refereeQueue = xQueueCreate(30, sizeof(RefereeMsg_s));
#endif

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
