#pragma once

#include "stdint.h"

static constexpr uint8_t CRC8_INIT = 0xFF;
static constexpr uint16_t CRC16_INIT = 0xFFFF;

// CRC8
void Append_CRC8_Check_Sum(uint8_t *_pchMessage, uint16_t _dwLength);

uint32_t Verify_CRC8_Check_Sum(const uint8_t *_pchMessage, uint16_t _dwLength);

uint8_t Get_CRC8_Check_Sum(const uint8_t *_pchMessage, uint16_t _dwLength, uint8_t _ucCRC8);

// CRC16
void Append_CRC16_Check_Sum(uint8_t *_pchMessage, uint32_t _dwLength);

uint32_t Verify_CRC16_Check_Sum(const uint8_t *_pchMessage, uint32_t _dwLength);

uint16_t Get_CRC16_Check_Sum(const uint8_t *_pchMessage, uint32_t _dwLength, uint16_t _wCRC);
