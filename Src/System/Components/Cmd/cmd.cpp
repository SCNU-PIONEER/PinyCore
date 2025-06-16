#include "cmd.hpp"
#include "stm_log.hpp"

#define EVENT_MASK (RTT_READY_EVENT | RC_READY_EVENT)

void Cmd::init()
{
    eventGroup = xEventGroupCreate();
    rttHandler.init(eventGroup);
    rcHandler.init(eventGroup);
}

void Cmd::addHandler(Handler *handler)
{
    if (handler) {
        handlerBus.push_back(handler);
    }
}

void Cmd::addObserver(Msg *_msg, IObserver *observer)
{
    msgBus[_msg].push_back(observer);
}

void Cmd::parseMsg()
{
    EventBits_t xBits = xEventGroupWaitBits(
            eventGroup, EVENT_MASK,
            pdTRUE,  // 是否在等待成功后清除事件位（pdTRUE 为清除）
            pdFALSE, // 是否需要所有事件位都被置位（pdTRUE 为全部）
            portMAX_DELAY);

    if (xBits & RTT_READY_EVENT) {
        rttHandler.handle();
    }

    if (xBits & RC_READY_EVENT) {
        rcHandler.handle();
    }

    notifyObservers();
    
    //TODO: check if the cmd is unvalid , disable all modules
}

inline void Cmd::notifyObservers()
{
    for (auto &pair : msgBus) {
        for (auto *observer : pair.second) {
            if (observer) {
                observer->getMsg(*pair.first);
            }
        }
    }
}
