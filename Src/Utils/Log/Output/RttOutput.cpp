#include "RttOutput.hpp"
#include "SEGGER_RTT.h"

namespace LOG {

bool RttOutput::init()
{
    SEGGER_RTT_Init();
    return true;
}

bool RttOutput::send(const uint8_t *_data, size_t _size)
{
    return SEGGER_RTT_Write(0, _data, static_cast<unsigned>(_size)) == _size;
}

} // namespace LOG
