#pragma once

#include <vector>
#include <unordered_map>
#include "handler.hpp"
#include "msgBase.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"

#include "rttMsgHandler.hpp"
#include "rcMsgHandler.hpp"

class IObserver {
public:
    virtual ~IObserver() = default;
    virtual void getMsg(Msg &_msg) = 0;
};

class Cmd {
public:
    void init();
    void addHandler(Handler *handler);
    void addObserver(Msg *_msg, IObserver *observer);
    void parseMsg();

protected:
    inline void notifyObservers();

private:
    std::vector<Handler *> handlerBus;
    std::unordered_map<Msg *, std::vector<IObserver *> > msgBus;
    EventGroupHandle_t eventGroup;

    RTTMsgHandler rttHandler;
    rcMsgHandler rcHandler;
};
