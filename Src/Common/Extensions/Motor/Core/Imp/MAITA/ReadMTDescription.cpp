#include "MTMotor.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace MTMOTOR;

void MTMotor::readErrorCode(std::array<uint8_t, 8> &_txBuf)
{
    constexpr std::array<uint8_t, 8> PACK = { 0x9A, 0, 0, 0, 0, 0, 0, 0 };
    _txBuf = PACK;
}

void MTMotor::readState2(std::array<uint8_t, 8> &_txBuf)
{
    constexpr std::array<uint8_t, 8> PACK = { 0x9C, 0, 0, 0, 0, 0, 0, 0 };
    _txBuf = PACK;
}

//TODO:添加脉塔其他的读取指令
