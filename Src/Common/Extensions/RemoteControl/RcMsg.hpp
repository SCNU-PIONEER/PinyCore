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

/* ----------RC Normalized Value Definition---------- */
#define RC_ROCKER_MAX ((uint8_t)1)
#define RC_SLIDER_MAX ((uint8_t)1)
#define RC_MOVE_MAX   ((uint8_t)1)
#define RC_ROLLER_MAX ((uint8_t)1)
/* ------------------Defined Marcos------------------*/

#define IS_KEY_PRESS(CODE, KEY) (((CODE) & (KEY)) == (KEY))
/* ------------------Data Struct Data Struct -------*/

enum class RcKb_e : uint8_t { W = 1u, S, A, D, SHIFT, CTRL, Q, E, R, F, G, Z, X, C, V, B };

enum class RcSw_e : uint8_t {
    DOWN = 0u,
    MID,
    UP,
};

typedef struct {
    struct {
        float rx;
        float ry;
        float lx;
        float ly;
    };

    struct {
        RcSw_e rSwitch;
        RcSw_e lSwitch;
    };

    struct {
        uint8_t rPress;
        uint8_t lPress;
        float xMove;
        float yMove;
        float zRoller;
    };

    float slider;

} RcMsg_t;
