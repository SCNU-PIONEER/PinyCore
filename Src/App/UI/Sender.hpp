/*
 * @file UISender.cpp
 * @breif: 常规链路 UI 串口发送
 * @note: 串口波特率为115200, 8位数据位，1位停止位，无硬件流控，无校验位,
 *        机器人交互数据，发送方触发发送，频率上限为 30 Hz, 数据段长度 127 bytes
 *        常规链路由裁判系统服务器和主控模块进行数据转发，从电源管理模块的 User 串口收发数据
 *        不需要图传发送端也能测试！UI 不是走图传链路
 * @link: https://www.robomaster.com/zh-CN/resource/pages/announcement/1768
 *
 * This file contains code from Priority_UI
 * Copyright (c) 2025 IsaacZH
 * Used under MIT License - https://opensource.org/licenses/MIT
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include "Crc.hpp"
#include "Bsp_uart.hpp"
#include "./Protocol.hpp"

namespace UI {

class Sender {
public:
    Sender(UART_HandleTypeDef _huart, uint8_t _id);

    /**
    * @brief 更新红蓝方机器人信息，在裁判系统接受中断中调用
    */
    void infoUpdate(uint16_t _id);

    uint8_t sendOneGraphic(OneGraphicData_s _data);
    uint8_t sendTwoGraphic(TwoGraphicData_s _data);
    uint8_t sendFiveGraphic(FiveGraphicData_s _data);
    uint8_t sendSevenGraphic(SevenGraphicData_s _data);
    uint8_t sendChar(CharGraphicData_s _data);
    uint8_t deleteGraphic(uint8_t _deleteLayer);

protected:
    void buildCommonFrame(uint8_t *_buf, const IDLen_e _dataLength,
                          const ID_e _cmdID);

    template <typename GraphicData_t>
    uint8_t sendGraphic(GraphicData_t &_data, const IDLen_e _len,
                        const ID_e _id, const uint8_t _graphicCount)
    {
        uint8_t buffer[128] = {};
        buildCommonFrame(buffer, _len, _id);
        memcpy(&buffer[13], _data.data, 15 * _graphicCount);
        uint16_t totalLen = 15 + (15 * _graphicCount);
        Append_CRC16_Check_Sum(buffer, totalLen);
        memcpy(txBuf_, buffer, totalLen);
        return send_(txBuf_, totalLen);
    }

private:
    UART_HandleTypeDef huart_;
    uint8_t *txBuf_; // DMA buffer for sending data
    std::function<uint8_t(uint8_t *, uint16_t)> send_;
    ClientInfo_s clientInfo_;
};

} // namespace UI
