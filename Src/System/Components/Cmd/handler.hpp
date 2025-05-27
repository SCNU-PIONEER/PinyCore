#pragma once

#include <cstdint>
#include "msgImpl.hpp"

class Handler {
public:
    Handler(chassisMsg *cmsg, gimbalMsg *gmsg) : cmsg(cmsg), gmsg(gmsg) {};

    /* get and parse data*/
    virtual uint8_t parseData() = 0;

    /* handle data to module msg*/
    virtual void handle() = 0;

protected:
    chassisMsg *cmsg;
    gimbalMsg *gmsg;
};


/* may be we can use factory pattern to create handlers */
class HandlerFactory {};
