/**
 * @file RefereeCrc.cpp
 * @brief 裁判系统CRC校验模块实现
 *
 * @version Version 1.0.0
 * @author Generated from C version
 * @date 2025/9/4
 *
 * @copyright SCNU-PIONEER (c) 2025-2026
 *
 */
#include "RefereeCrc.hpp"

namespace REFEREE {

uint8_t RefereeCrc::getCRC8CheckSum(const uint8_t *pchMessage,
                                    uint16_t dwLength, uint8_t ucCRC8)
{
    uint8_t ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = crc8Tab_[ucIndex];
    }
    return ucCRC8;
}

bool RefereeCrc::verifyCRC8CheckSum(const uint8_t *pchMessage,
                                    uint16_t dwLength)
{
    uint8_t ucExpected = 0;

    if (pchMessage == nullptr || dwLength <= 2) {
        return false;
    }

    ucExpected = getCRC8CheckSum(pchMessage, dwLength - 1, CRC8_INIT);

    return (ucExpected == pchMessage[dwLength - 1]);
}

void RefereeCrc::appendCRC8CheckSum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucCRC = 0;

    if (pchMessage == nullptr || dwLength <= 2) {
        return;
    }

    ucCRC = getCRC8CheckSum(pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}

uint16_t RefereeCrc::getCRC16CheckSum(const uint8_t *pchMessage,
                                      uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;

    if (pchMessage == nullptr) {
        return 0xFFFF;
    }

    while (dwLength--) {
        chData = *pchMessage++;
        wCRC = ((uint16_t)(wCRC) >> 8) ^
               crc16Tab_[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }

    return wCRC;
}

bool RefereeCrc::verifyCRC16CheckSum(const uint8_t *pchMessage,
                                     uint32_t dwLength)
{
    uint16_t wExpected = 0;

    if (pchMessage == nullptr || dwLength <= 2) {
        return false;
    }

    wExpected = getCRC16CheckSum(pchMessage, dwLength - 2, CRC16_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] &&
            ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void RefereeCrc::appendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;

    if (pchMessage == nullptr || dwLength <= 2) {
        return;
    }

    wCRC = getCRC16CheckSum(pchMessage, dwLength - 2, CRC16_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}

} // namespace REFEREE
