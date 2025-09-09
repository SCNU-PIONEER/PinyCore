/**
 * @file rc_msg.hpp
 * @brief 遥控器数据集
 *
 * @version Version 1.0.1
 * @author yjy
 * @date 2025/9/8
 *
 * @copyright SCNU-PIONEER (c) 2025-2026
 *
 */
#pragma once

#include <cstdint>

/* --------------- RC Channel Definition-----------------*/
#define RC_CH_VALUE_MIN    ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX    ((uint16_t)1684)
#define RC_CH_VALUE_RANGE  ((uint16_t)660)
/* ----------RC Switch Definition------------------------- */
#define RC_SW_UP           ((uint8_t)1)
#define RC_SW_MID          ((uint8_t)3)
#define RC_SW_DOWN         ((uint8_t)2)
/* ----------PC Key Definition---------------------- ------*/
#define MOUSE_PRESS        ((uint8_t)1)
#define MOUSE_NOT_PRESS    ((uint8_t)0)
#define MOUSE_MAX_ABS      ((int16_t)32768)
#define KEY_W              ((uint16_t)0x01 << 0)
#define KEY_S              ((uint16_t)0x01 << 1)
#define KEY_A              ((uint16_t)0x01 << 2)
#define KEY_D              ((uint16_t)0x01 << 3)
#define KEY_SHIFT          ((uint16_t)0x01 << 4)
#define KEY_CTRL           ((uint16_t)0x01 << 5)
#define KEY_Q              ((uint16_t)0x01 << 6)
#define KEY_E              ((uint16_t)0x01 << 7)
#define KEY_R              ((uint16_t)0x01 << 8)
#define KEY_F              ((uint16_t)0x01 << 9)
#define KEY_G              ((uint16_t)0x01 << 10)
#define KEY_Z              ((uint16_t)0x01 << 11)
#define KEY_X              ((uint16_t)0x01 << 12)
#define KEY_C              ((uint16_t)0x01 << 13)
#define KEY_V              ((uint16_t)0x01 << 14)
#define KEY_B              ((uint16_t)0x01 << 15)
/* ------------------Defined Errors------------------*/
#define RC_NO_ERROR        0
#define RC_CH_ERROR        0xFF
#define RC_VERIFY_ERR      0xFE
#define RC_RX_LOST_MAX     ((uint8_t)15)

/* ------------------Defined Marcos------------------*/

#define IS_KEY_PRESS(CODE, KEY) (((CODE) & (KEY)) == (KEY))
/* ------------------Data Struct Data Struct -------*/
#define RC_FRAME_LENGTH         18

namespace RC {

enum KeyCodes_e : uint8_t {
    W = 1,
    S = 2,
    A = 3,
    D = 4,
    SHIFT = 5,
    CTRL = 6,
    Q = 7,
    E = 8,
    R = 9,
    F = 10,
    G = 11,
    Z = 12,
    X = 13,
    C = 14,
    V = 15,
    B = 16
};

typedef struct {
    struct {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        uint8_t switchLeft;
        uint8_t switchRight;
    } rc;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t pressLeft;
        uint8_t pressRight;
    } mouse;
    struct {
        uint16_t keyCode;
        uint16_t lastKeyCode;
    } keyboard;
    int16_t wheel;
} RcRawMsg_t;

} // namespace RC
