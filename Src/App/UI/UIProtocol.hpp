/*
 * @file UIProtocol.cpp
 * @breif: 常规链路 UI 串口协议
 * @note: 串口波特率为115200, 8位数据位，1位停止位，无硬件流控，无校验位,
 *        机器人交互数据，发送方触发发送，频率上限为 30 Hz, 数据段长度 127 bytes
 *        常规链路由裁判系统服务器和主控模块进行数据转发，从电源管理模块的 User 串口收发数据
 *        不需要图传也能测试！UI 不是走图传链路
 * @link: https://www.robomaster.com/zh-CN/resource/pages/announcement/1768
 *
 * This file contains code from Priority_UI
 * Copyright (c) 2025 IsaacZH
 * Used under MIT License - https://opensource.org/licenses/MIT
 */

#pragma once

#include <cstdint>

namespace UI {

/*
 * @breif 子内容ID
 */
enum class ID_e : uint16_t {
    // 0x200-0x02ff 	队伍自定义命令 格式  INTERACT_ID_XXXX
    DELETE = 0x0100,     // 客户端删除图形
    DRAW_ONE = 0x0101,   // 客户端绘制一个图形
    DRAW_TWO = 0x0102,   // 客户端绘制二个图形
    DRAW_FIVE = 0x0103,  // 客户端绘制五个图形
    DRAW_SEVEN = 0x0104, // 客户端绘制七个图形
    DRAW_CHAR = 0x0110,  // 客户端绘制字符图形
};

/*
 * @breif 子内容数据段长度
 */
// NOLINTNEXTLINE(performance-enum-size)
enum class IDLen_e : uint16_t {
    DELETE = 8,       // 6+2
    DRAW_ONE = 21,    // 6+15
    DRAW_TWO = 36,    // 6+15*2
    DRAW_FIVE = 81,   // 6+15*5
    DRAW_SEVEN = 111, // 6+15*7
    DRAW_CHAR = 51,   // 6+15+30（字符串内容）
};

/*
 * @breif 子内容图形操作
 */
// NOLINTNEXTLINE(performance-enum-size)
enum OperateType_e {
    NONE,   // 空操作
    ADD,    // 增加图层
    MODIFY, // 修改图层
    DELETE, // 删if除图层
};

/*
 * @breif 子内容图形类型
 */
// NOLINTNEXTLINE(performance-enum-size)
enum class GraphicType_e : uint32_t {
    LINE,      // 直线
    RECTANGLE, // 矩形
    CIRCLE,    // 圆
    ELLIPSE,   // 椭圆
    ARC,       // 圆弧
    FLOAT,     // 浮点数
    INT,       // 整型数
    CHAR,      // 字符
};

/*
 * @breif 子内容颜色
 */
enum class Color_e : uint8_t {
    RED_BLUE, // 红蓝主色
    YELLOW,
    GREEN,
    ORANGE,
    FUCHSIA, // 紫红色
    PINK,
    CYAN_BLUE, // 青色
    BLACK,
    WHITE
};

#pragma pack(push, 1)
/*
 * 角度值含义为：0°指 12 点钟方向，顺时针绘制
 * 屏幕位置：（0,0）为屏幕左下角（1920，1080）为屏幕右上角
 * 浮点数：整型数均为 32 位，对于浮点数，实际显示的值为输入的值/1000,
 * 如在输入 1234，显示的值将为 1.234。
*/
struct GraphicData_s {
    uint8_t figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;
};

/*
 * @breif 子内容ID 0x100
 */
struct DeleteGraphicData_s {
    uint8_t operateTpye;
    uint8_t layer;
};

/*
 * @breif 子内容ID 0x101
 */
struct OneGraphicData_s {
    GraphicData_s data[1];
};

/*
 * @breif 子内容ID 0x102
 */
struct TwoGraphicData_s {
    GraphicData_s data[2];
};

/*
 * @breif 子内容ID 0x102
 */
struct FiveGraphicData_s {
    GraphicData_s data[5];
};

/*
 * @breif 子内容ID 0x103
 */
struct SevenGraphicData_s {
    GraphicData_s data[7];
};

/*
 * @breif 子内容ID 0x103
 */
struct CharGraphicData_s {
    GraphicData_s option;
    char string[30];
};

struct FrameHeader_s {
    uint8_t SOF = 0xA5; // 1byte, 数据帧起始字节，固定值为 0xA5
    IDLen_e dataLength; // 2byte, 数据帧中 data 的长度
    uint8_t seq;        // 1byte, 包序号
    uint8_t CRC8;       // 1byte, 帧头 CRC8 校验
};

struct DataHeader_s {
    ID_e cmdID;          // 2byte, 内容ID
    uint16_t senderID;   // 2byte, 发送者的 ID
    uint16_t receiverID; // 2byte, 接收者的 ID
};

/*
 * @breif: 机器人间交互数据帧
 * @note: 数据段头结构包括内容 ID、发送者和接收者的 ID、内容数据段。
 *        机器人交互数据包的总长不超过 127 个字节，减去 frame_header、
 *        cmd_id 和 frame_tail 的 9 个字节以及数据段头结构的 6 个字节，
 *        故机器人交互数据的内容数据段最大为 112 个字节,每 1000 毫秒，
 *        英雄、工程、步兵、空中机器人、飞镖能够接收数据的上限为 3720 字节
 *        雷达和哨兵机器人能够接收数据的上限为 5120 字节
 * */
struct Frame_s {
    FrameHeader_s header;    // 5byte, 帧头
    uint16_t cmdID = 0x301;  // 2byte, 命令码 ID
    DataHeader_s dataHeader; // 6byte, 数据段头结构
    uint16_t tail;           // 2byte, 帧尾 CRC16，整包校验
};

/* 客户端信息 */
struct ClientInfo_s {
    uint8_t robotID;
    uint16_t clientID;
};
#pragma pack(pop)

} // namespace UI
