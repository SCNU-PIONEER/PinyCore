#include "Cmd.hpp"
#include "MsgImpl.hpp"
#include "StmLog.hpp"

Cmd::Cmd()
        : Task<Cmd, 256>("CmdTask", TaskPriority_e::LOW2)
        , eventGroup_(xEventGroupCreate())
        , rcHandler_(&RC_UART, eventGroup_)

{
    msgBus_.chassisQueue = xQueueCreate(30, sizeof(ChassisMsg_s));
    msgBus_.gimbalQueue = xQueueCreate(30, sizeof(GimbalMsg_s));
    msgBus_.armQueue = xQueueCreate(30, sizeof(ArmMsg_s));
    msgBus_.refereeQueue = xQueueCreate(30, sizeof(RefereeMsg_s));

    masks_.reset();

    rttHandler_.init(&msgBus_, eventGroup_);
    masks_ = masks_ | std::bitset<32>(RTT_READY_EVENT);

    rcHandler_.init(&msgBus_, eventGroup_);
    masks_ = masks_ | std::bitset<32>(RC_READY_EVENT);
    masks_ = masks_ | std::bitset<32>(ET08A_READY_EVENT);

#if defined APP_USE_REFEREE
    refereeHandler_.init(&msgBus_, eventGroup_);
    masks_ = masks_ | std::bitset<32>(REFEREE_READY_EVENT);
#endif

    LOG::info("cmd", "init success");
}


void Cmd::parseMsg()
{
    EventBits_t xBits = xEventGroupWaitBits(eventGroup_, masks_.to_ulong(),
                                            pdTRUE, pdFALSE, portMAX_DELAY);
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

void Cmd::task()
{
    for (;;) {
        parseMsg();
    }
}
