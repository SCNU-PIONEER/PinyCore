/**
 * @file Referee.cpp
 * @brief 裁判系统实现
 *
 * @version Version 1.0.0
 * @author Generated from C version
 * @date 2025/9/3
 *
 * @copyright SCNU-PIONEER (c) 2025-2026
 *
 */
#include "Referee.hpp"
#include "RefereeCrc.hpp"
#include "Bsp_uart.hpp"
#include "Bsp_dma.hpp"
#include "sdkconfig.h"
#include <cstring>



namespace REFEREE {

Referee::Referee() 
    : rxBuffer_(nullptr), uart_(nullptr), rxLostCnt_(REFEREE_SYS_MAX_LOST), previousOffset_(0) {
    std::memset(&gameStatus_, 0, sizeof(gameStatus_));
    std::memset(&gameResult_, 0, sizeof(gameResult_));
    std::memset(&gameRobotHP_, 0, sizeof(gameRobotHP_));
    std::memset(&eventData_, 0, sizeof(eventData_));
    std::memset(&supplyProjectileAction_, 0, sizeof(supplyProjectileAction_));
    std::memset(&refereeWarning_, 0, sizeof(refereeWarning_));
    std::memset(&dartRemainingTime_, 0, sizeof(dartRemainingTime_));
    std::memset(&gameRobotStatus_, 0, sizeof(gameRobotStatus_));
    std::memset(&powerHeatData_, 0, sizeof(powerHeatData_));
    std::memset(&gameRobotPos_, 0, sizeof(gameRobotPos_));
    std::memset(&buff_, 0, sizeof(buff_));
    std::memset(&aerialRobotEnergy_, 0, sizeof(aerialRobotEnergy_));
    std::memset(&robotHurt_, 0, sizeof(robotHurt_));
    std::memset(&shootData_, 0, sizeof(shootData_));
    std::memset(&bulletRemaining_, 0, sizeof(bulletRemaining_));
    std::memset(&rfidStatus_, 0, sizeof(rfidStatus_));
    
    std::memset(txBuffer_, 0, sizeof(txBuffer_));
    std::memset(cmdidRxTable_, 0, sizeof(cmdidRxTable_));
    
    initCmdidTable();
}

void Referee::initCmdidTable() {
    // 初始化数据指针表
    std::memset(cmdidBufferTable_, 0, sizeof(cmdidBufferTable_));
    
    // 第0行 (0x00xx)
    cmdidBufferTable_[0][1] = &gameStatus_;      // 0x0001
    cmdidBufferTable_[0][3] = &gameRobotHP_;     // 0x0003
    
    // 第1行 (0x01xx)  
    cmdidBufferTable_[1][2] = &supplyProjectileAction_; // 0x0102
    cmdidBufferTable_[1][4] = &refereeWarning_;         // 0x0104
    cmdidBufferTable_[1][5] = &dartRemainingTime_;      // 0x0105
    
    // 第2行 (0x02xx)
    cmdidBufferTable_[2][1] = &gameRobotStatus_;   // 0x0201
    cmdidBufferTable_[2][2] = &powerHeatData_;     // 0x0202
    cmdidBufferTable_[2][3] = &gameRobotPos_;      // 0x0203
    cmdidBufferTable_[2][4] = &buff_;              // 0x0204
    cmdidBufferTable_[2][5] = &aerialRobotEnergy_; // 0x0205
    cmdidBufferTable_[2][6] = &robotHurt_;         // 0x0206
    cmdidBufferTable_[2][7] = &shootData_;         // 0x0207
    cmdidBufferTable_[2][8] = &bulletRemaining_;   // 0x0208
    cmdidBufferTable_[2][9] = &rfidStatus_;        // 0x0209
}

void Referee::init(UART_HandleTypeDef *_huart) {
    uart_ = _huart;
    rxBuffer_ = (uint8_t *)Dma::instance().ram_alloc(REFEREE_RX_BUFFER_LEN * 2);
    
    Uart::instance().RecvDmaMultiBufInit(_huart, (uint32_t *)&rxBuffer_[0], 
                                         REFEREE_RX_BUFFER_LEN * 2);
    Uart::instance().registerCallback(_huart, [this](UART_HandleTypeDef *_huart, uint16_t _size) {
        this->uartIdleCallback(_huart);
    });
    
    // 启用UART空闲中断
    __HAL_UART_ENABLE_IT(_huart, UART_IT_IDLE);
}

void Referee::rawUartIdleCallback(UART_HandleTypeDef *_huart) {
    Referee::instance().uartIdleCallback(_huart);
}

void Referee::uartIdleCallback(UART_HandleTypeDef *_huart) {
    if (_huart->Instance != uart_->Instance) {
        return;
    }
    
    uint32_t currentOffset, len;
    
    // 计算当前DMA接收位置
    currentOffset = REFEREE_RX_BUFFER_LEN * 2 - __HAL_DMA_GET_COUNTER(_huart->hdmarx);
    
    if (currentOffset >= previousOffset_) {
        len = currentOffset - previousOffset_;
    } else {
        len = REFEREE_RX_BUFFER_LEN * 2 - previousOffset_ + currentOffset;
    }
    
    if (len > 0) {
        // 处理接收的数据
        if (previousOffset_ + len <= REFEREE_RX_BUFFER_LEN * 2) {
            // 数据连续
            previousOffset_ = readRefereeData(rxBuffer_, previousOffset_, len);
        } else {
            // 数据回绕，分两次处理
            uint32_t firstPart = REFEREE_RX_BUFFER_LEN * 2 - previousOffset_;
            readRefereeData(rxBuffer_, previousOffset_, firstPart);
            previousOffset_ = readRefereeData(rxBuffer_, 0, len - firstPart);
        }
        
        if (previousOffset_ >= REFEREE_RX_BUFFER_LEN * 2) {
            previousOffset_ -= REFEREE_RX_BUFFER_LEN * 2;
        }
        
        rxLostCnt_ = 0;
    } else {
        incRxLost();
    }
}

uint32_t Referee::readRefereeData(uint8_t *_buf, uint32_t _startOffset, uint32_t _len) {
    uint32_t nextStart, curStart, curFrameLen;
    uint32_t maxOffset = _len + _startOffset;
    
    for (curStart = _startOffset; curStart < maxOffset; curStart = nextStart) {
        // 查找帧头
        while (curStart < maxOffset && _buf[curStart] != SOF_BYTE) {
            curStart++;
        }
        
        if (curStart >= maxOffset) {
            break;
        }
        
        // 检查是否有足够的数据读取帧头
        if (curStart + sizeof(FrameHeader_s) > maxOffset) {
            break;
        }
        
        FrameHeader_s *header = (FrameHeader_s *)&_buf[curStart];
        curFrameLen = sizeof(FrameHeader_s) + header->dataLen + LEN_TAIL;
        nextStart = curStart + curFrameLen;
        
        // 检查帧长度是否合理
        if (header->dataLen > 128 || nextStart > maxOffset) {
            nextStart = curStart + 1;
            continue;
        }
        
        // 验证帧头CRC8
        if (!RefereeCrc::verifyCRC8CheckSum((uint8_t *)header, sizeof(FrameHeader_s))) {
            nextStart = curStart + 1;
            continue;
        }
        
        // 验证整帧CRC16
        if (!RefereeCrc::verifyCRC16CheckSum(&_buf[curStart], curFrameLen)) {
            nextStart = curStart + 1;
            continue;
        }
        
        // 获取命令ID
        uint16_t cmdId = *((uint16_t *)&_buf[curStart + sizeof(FrameHeader_s)]);
        
        // 查找对应的数据结构
        uint8_t highByte = (cmdId >> 8) & 0x0F;
        uint8_t lowByte = cmdId & 0xFF;
        
        if (highByte < CMDID_TABLE_ROW && lowByte < CMDID_TABLE_COL) {
            void *dataPtr = cmdidBufferTable_[highByte][lowByte];
            if (dataPtr != nullptr) {
                // 复制数据
                std::memcpy(dataPtr, &_buf[curStart + sizeof(FrameHeader_s) + LEN_CMDID], 
                           header->dataLen - LEN_CMDID);
                
                // 设置接收标志
                cmdidRxTable_[highByte] |= (1 << lowByte);
            }
        }
    }
    
    return curStart;
}

uint16_t Referee::sendSingleFrame(uint16_t _cmdId, uint8_t *_pStruct, uint16_t _len) {
    uint16_t totalSize;
    FrameHeader_s txHeader;
    
    if (_len + sizeof(FrameHeader_s) + 4 > REFEREE_TX_BUFFER_LEN) {
        return 0;
    }
    
    std::memset(txBuffer_, 0, REFEREE_TX_BUFFER_LEN);
    
    txHeader.sof = SOF_BYTE;
    txHeader.dataLen = _len + LEN_CMDID;
    txHeader.seq = 0;
    
    // 计算帧头CRC8
    RefereeCrc::appendCRC8CheckSum((uint8_t *)&txHeader, sizeof(FrameHeader_s));
    
    // 复制帧头
    std::memcpy(txBuffer_, &txHeader, sizeof(FrameHeader_s));
    
    // 复制命令ID
    std::memcpy(&txBuffer_[sizeof(FrameHeader_s)], &_cmdId, LEN_CMDID);
    
    // 复制数据
    std::memcpy(&txBuffer_[sizeof(FrameHeader_s) + LEN_CMDID], _pStruct, _len);
    
    totalSize = sizeof(FrameHeader_s) + _len + LEN_CMDID;
    
    // 计算整帧CRC16
    RefereeCrc::appendCRC16CheckSum(txBuffer_, totalSize + LEN_TAIL);
    totalSize += LEN_TAIL;
    
    // 发送数据
    HAL_UART_Transmit_DMA(uart_, txBuffer_, totalSize);
    
    return totalSize;
}

uint16_t Referee::getRxFlags(uint16_t _cmdId) {
    uint8_t highByte = (_cmdId >> 8) & 0x0F;
    uint8_t lowByte = _cmdId & 0xFF;
    
    if (highByte < CMDID_TABLE_ROW) {
        return cmdidRxTable_[highByte] & (1 << lowByte);
    }
    return 0;
}

void Referee::clearRxFlags(uint16_t _cmdId) {
    uint8_t highByte = (_cmdId >> 8) & 0x0F;
    uint8_t lowByte = _cmdId & 0xFF;
    
    if (highByte < CMDID_TABLE_ROW) {
        cmdidRxTable_[highByte] &= ~(1 << lowByte);
    }
}

void Referee::incRxLost() {
    if (rxLostCnt_ < REFEREE_SYS_MAX_LOST) {
        rxLostCnt_++;
    }
}

bool Referee::isOffline() {
    return rxLostCnt_ >= REFEREE_SYS_MAX_LOST;
}

void Referee::clearBuffer() {
    __HAL_UART_DISABLE_IT(uart_, UART_IT_IDLE);
    std::memset(cmdidRxTable_, 0, sizeof(cmdidRxTable_));
    __HAL_UART_ENABLE_IT(uart_, UART_IT_IDLE);
}

} // namespace REFEREE
