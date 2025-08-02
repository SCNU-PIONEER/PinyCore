#pragma once

#include "Rc.hpp"

// 臂气泵
#define PUMP_1_TOGGLE  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15)
#define PUMP_1_ON      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET)
#define PUMP_1_OFF     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET)

// 吸盘气泵
#define PUMP_2_TOGGLE  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3)
#define PUMP_2_ON      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define PUMP_2_OFF     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)

// 臂电磁阀1
#define VALVE_1_TOGGLE HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4)
#define VALVE_1_ON     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define VALVE_1_OFF    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)

// 臂电磁阀2
#define VALVE_2_TOGGLE HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_7)
#define VALVE_2_ON     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET)
#define VALVE_2_OFF    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET)

enum class PumpState_e : uint8_t {
    OFF = 0,
    ON = 1
};

struct Pump_s {
    PumpState_e state[4]{PumpState_e::OFF, PumpState_e::OFF, PumpState_e::OFF, PumpState_e::OFF};
};

void pumpInit();
void pumpCtrl(const Pump_s& _pump);

extern Pump_s silverPumpTest[1];
extern Pump_s silver_left_pump[7];
extern Pump_s all_pump_on[1];
extern Pump_s all_pump_off[1];