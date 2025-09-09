/*
 * @file Sender.cpp
 * @breif: 常规链路 UI 串口协议
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

#include <cstdint>
#include "./UISender.hpp"
#include "Bsp_dma.hpp"

using namespace UI;

Sender::Sender(UART_HandleTypeDef _huart, uint8_t _id)
        : huart_(_huart)
        , txBuf_((uint8_t *)Dma::instance().ram_alloc(128))
        , clientInfo_{ .robotID = _id,
                       .clientID = static_cast<uint16_t>(_id + 0x100) }

{
    send_ = [this](uint8_t *_data, uint16_t _len) -> uint8_t {
        return HAL_UART_Transmit_DMA(&huart_, _data, _len);
    };
}

void Sender::infoUpdate(uint16_t _id)
{
    clientInfo_.robotID = _id;
    clientInfo_.clientID = _id + 0x100;
}

void Sender::buildCommonFrame(uint8_t *_buf, const IDLen_e _dataLength,
                              const ID_e _cmdID)
{
    Frame_s frame = {};
    frame.header.SOF = 0xA5;
    frame.header.dataLength = _dataLength;
    frame.header.seq = 0;
    memcpy(_buf, &frame.header, 4);
    Append_CRC8_Check_Sum(_buf, 5);

    frame.cmdID = 0x301;
    memcpy(&_buf[5], (void *)&frame.cmdID, 2);

    DataHeader_s dataHeader;
    dataHeader.cmdID = _cmdID;
    dataHeader.senderID = clientInfo_.robotID;
    dataHeader.receiverID = clientInfo_.clientID;
    memcpy(&_buf[7], &dataHeader, 6);
}


uint8_t Sender::sendOneGraphic(OneGraphicData_s _data)
{
    return sendGraphic(_data, IDLen_e::DRAW_ONE, ID_e::DRAW_ONE, 1);
}

uint8_t Sender::sendTwoGraphic(TwoGraphicData_s _data)
{
    return sendGraphic(_data, IDLen_e::DRAW_TWO, ID_e::DRAW_TWO, 2);
}

uint8_t Sender::sendFiveGraphic(FiveGraphicData_s _data)
{
    return sendGraphic(_data, IDLen_e::DRAW_FIVE, ID_e::DRAW_FIVE, 5);
}

uint8_t Sender::sendSevenGraphic(SevenGraphicData_s _data)
{
    return sendGraphic(_data, IDLen_e::DRAW_SEVEN, ID_e::DRAW_SEVEN, 7);
}

uint8_t Sender::sendChar(CharGraphicData_s _data)
{
    uint8_t buffer[128];
    buildCommonFrame(buffer, IDLen_e::DRAW_CHAR, ID_e::DRAW_CHAR);
    memcpy(&buffer[13], &_data.option, 15);
    memcpy(&buffer[28], _data.string, 30);
    Append_CRC16_Check_Sum(buffer, 60);
    memcpy(txBuf_, buffer, 60);
    return send_(txBuf_, 60);
}

uint8_t Sender::deleteGraphic(uint8_t _deleteLayer)
{
    uint8_t buffer[128];
    buildCommonFrame(buffer, IDLen_e::DRAW_CHAR, ID_e::DELETE);
    buffer[13] = 1;            // 删除单个图层
    buffer[14] = _deleteLayer; // 删除图层
    Append_CRC16_Check_Sum(buffer, 17);
    memcpy(txBuf_, buffer, 17);
    return send_(txBuf_, 5 + 2 + 8 + 2);
}
