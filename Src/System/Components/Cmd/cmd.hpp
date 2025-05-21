#pragma once

#include <vector>
#include "rc.hpp"

#include "./chassisMsg.hpp"


class IObserver {
public:
    virtual ~IObserver() = default;
    virtual void update(Msg &_msg) = 0;
};


class Cmd {
public:
    Cmd() {};

    void init(UART_HandleTypeDef *uart) { rc_.init(uart); };

    void addObserver(IObserver *observer) { observers.push_back(observer); }

    void task();

    inline void update()
    {
        if (rc_.parseData() != RC_VERIFY_ERR) {
            parse();
            notifyObservers();
        }
        //TODO: check if the cmd is unvalid , disable all modules
    }

protected:
    void parse();

    inline void notifyObservers()
    {
        for (auto *observer : observers) {
            if (observer != nullptr) {
                observer->update(msg);
            }
        }
    }


private:
    RC::Rc &rc_ = RC::Rc::instance();

    std::vector<IObserver *> observers;

    chassisMsg msg;
};
