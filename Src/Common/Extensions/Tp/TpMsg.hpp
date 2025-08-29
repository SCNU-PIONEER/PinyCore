#pragma once
#include "sdkconfig.h"
#include <cstdint>

/**
 * @brief 示教器控制数据结构
 * @details 包含7轴关节数据及状态控制位
 */
typedef struct {
    float joint[7]; // 7个关节角度值 (28字节)
    uint8_t push;   // 推送状态标志位
    uint8_t state1; // 状态位1
} TpCtrl_t;         // 总长度：30字节

/**
 * @brief 示教器帧数据结构
 * @details 包含帧头、命令ID、控制数据及帧尾校验
 */
typedef struct {
    // frame_header_t frameHeader;  // 帧头信息
    uint16_t cmdId;     // 命令ID
    TpCtrl_t tpData;    // 示教器控制数据
    uint16_t frameTail; // 整包校验值（CRC16等）
} VtTpFrame_t;
