#pragma once

#include "msgBase.hpp"

class IObserver {
public:
    virtual ~IObserver() = default;
    virtual void getMsg(Msg& _msg) = 0;
};
