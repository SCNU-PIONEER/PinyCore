#include "Handler.hpp"

std::vector<Handler::HandlerItem_s> &Handler::getHandlerList()
{
    static std::vector<HandlerItem_s> handlerList;
    return handlerList;
}

std::bitset<32> &Handler::getMasks()
{
    static std::bitset<32> masks = 0;
    return masks;
}

Handler::Handler() : bit_(allocateBit())
{
    auto &handlerList = getHandlerList();
    auto &masks = getMasks();

    handlerList.push_back({ bit_, this });
    masks = masks | std::bitset<32>(bit_);
}

uint32_t Handler::allocateBit()
{
    // bit must less than 24 or 32 which depends on the configuration of FreeRTOS
    static uint32_t nextBit = 0;

    uint32_t bit = (1u << nextBit);
    nextBit++;
    return bit;
}