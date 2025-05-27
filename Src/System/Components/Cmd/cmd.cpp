#include "cmd.hpp"

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
    for (auto *handler : handlerBus) {
        if (handler) {
            handler->parseData();
            handler->handle();

            notifyObservers();
        }
    }

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
