
#include "Pump.hpp"
#include "StmLog.hpp"
#include "Rc.hpp"

void pumpInit()
{
    VALVE_1_OFF; // 臂电磁阀1
    VALVE_2_OFF; // 臂电磁阀2
    PUMP_1_OFF;  // 臂气泵
    PUMP_2_OFF;  // 吸盘气泵

    LOG::Logger::instance().info(LOCATION, "Pump", "init");
}

void pumpCtrl(const Pump_s& _pump)
{
    if (_pump.state[0] == PumpState_e::ON) {
        PUMP_1_ON;
    } else {
        PUMP_1_OFF;
    }

    if (_pump.state[1] == PumpState_e::ON) {
        PUMP_2_ON;
    } else {
        PUMP_2_OFF;
    }

    if (_pump.state[2] == PumpState_e::ON) {
        VALVE_1_ON;
    } else {
        VALVE_1_OFF;
    }

    if (_pump.state[3] == PumpState_e::ON) {
        VALVE_2_ON;
    } else {
        VALVE_2_OFF;
    }
}

// 控制四个IO，1-臂气泵，2-吸盘气泵，3-臂电磁阀1，4-臂电磁阀2

Pump_s silverPumpTest[1] = { { PumpState_e::ON, PumpState_e::OFF, PumpState_e::ON, PumpState_e::ON } };

Pump_s silver_left_pump[7] = {
    { PumpState_e::OFF, PumpState_e::OFF, PumpState_e::OFF, PumpState_e::OFF },
    { PumpState_e::ON,  PumpState_e::ON, PumpState_e::ON,  PumpState_e::ON  },
    { PumpState_e::ON, PumpState_e::ON,  PumpState_e::ON, PumpState_e::ON  },
    { PumpState_e::ON,  PumpState_e::ON,  PumpState_e::ON, PumpState_e::ON  },
    { PumpState_e::ON, PumpState_e::ON, PumpState_e::ON,  PumpState_e::ON  },
    { PumpState_e::ON,  PumpState_e::ON, PumpState_e::ON, PumpState_e::ON  },
    { PumpState_e::ON, PumpState_e::ON,  PumpState_e::ON,  PumpState_e::ON  }
};

Pump_s all_pump_on[1]  = { { PumpState_e::ON, PumpState_e::ON, PumpState_e::ON, PumpState_e::ON } };
Pump_s all_pump_off[1] = { { PumpState_e::OFF, PumpState_e::OFF, PumpState_e::OFF, PumpState_e::OFF } };

// void Pump::ctrl(rc_ctrl_t *rc)
// {
// if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
//     store_pump[0] = 1;
// } else {
//     store_pump[0] = 0;
// }
// if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET) {
//     store_pump[1] = 1;
// } else {
//     store_pump[1] = 0;
// }
//
// if (is_key_pressed(KEY_CTRL)) {
//     if (is_key_last_pressed(KEY_G) && !is_key_pressed(KEY_G)) // ctrl + g - 左矿仓
//     {
//         PORT_A_TOG;
//     }
//     if (is_key_last_pressed(KEY_B) && !is_key_pressed(KEY_B)) // ctrl + b - 右矿仓
//     {
//         PORT_B_TOG;
//     }
//     if (is_key_last_pressed(KEY_R) && !is_key_pressed(KEY_R)) // ctrl + r - 开启泵
//     {
//         PORT_D_TOG;
//     }
//     if (is_key_last_pressed(KEY_F) && !is_key_pressed(KEY_F)) //  ctrl + f  -  金矿电磁阀
//     {
//         PORT_E_TOG;
//     }
//     if (is_key_last_pressed(KEY_V) && !is_key_pressed(KEY_V)) // ctrl + v  - 兑矿电磁阀
//     {
//         PORT_F_TOG;
//     }
//     if (is_key_pressed(KEY_C)) {
//         PORT_A_DE;
//         PORT_B_DE;
//         PORT_D_DE;
//         PORT_E_DE;
//         PORT_F_DE;
//     }
// }
//
// if (rc->rc.switch_right != RC_SW_DOWN) {
//     // if (cus_data.ctrl_state == customer) {
//     //     if (RAW_BUT_PUMP == 1)
//     //         PORT_F_EN;
//     //     else if (RAW_BUT_PUMP == 0)
//     //         PORT_F_EN;
//     // }
// }
// if (rc->rc.switch_right == RC_SW_DOWN && rc->rc.switch_left == RC_SW_MID) {
//     if (rc->rc.ch3 <= -650) {
//         PORT_A_EN; // A - B 是气泵
//         PORT_B_EN;
//         PORT_C_EN; // C - F 是电磁阀
//         PORT_D_EN;
//         PORT_E_EN;
//         PORT_F_EN;
//     } else if (rc->rc.ch1 <= -650) {
//         PORT_C_EN;
//         PORT_D_EN;
//         PORT_E_EN;
//         PORT_F_EN;
//     } else if (rc->rc.ch1 >= 650) {
//         PORT_A_EN;
//         PORT_B_EN;
//     } else if (rc->rc.ch3 != -660 && rc->rc.ch1 != 660 && rc->rc.ch1 != -660) {
//         PORT_A_DE;
//         PORT_B_DE;
//         PORT_C_DE;
//         PORT_D_DE;
//         PORT_E_DE;
//         PORT_F_DE;
//     }
// }
// }
