#include "Cmd.hpp"
#include "MsgImpl.hpp"
#include "StmLog.hpp"

Cmd::Cmd() : Task<Cmd, 256>("CmdTask", TaskPriority_e::LOW2)

{
    msgBus_.gimbalQueue = xQueueCreate(30, sizeof(GimbalMsg_s));
    msgBus_.shooterQueue = xQueueCreate(30, sizeof(ShooterMsg_s));

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
