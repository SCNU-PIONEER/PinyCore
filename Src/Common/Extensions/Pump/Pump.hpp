#pragma once

#include "Rc.hpp"
#include <cstdint>

namespace PUMP {
// 硬件控制宏定义
#define PUMP_1_TOG  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15)
#define PUMP_1_ON   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET)
#define PUMP_1_OFF  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET)

#define PUMP_2_TOG  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3)
#define PUMP_2_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define PUMP_2_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)

#define VALVE_1_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4)
#define VALVE_1_ON  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define VALVE_1_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)

#define VALVE_2_TOG HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_7)
#define VALVE_2_ON  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET)
#define VALVE_2_OFF HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET)

// 设备类型枚举
enum class Device_e : uint8_t { PUMP_1, PUMP_2, VALVE_1, VALVE_2, COUNT };
enum class State_e : uint8_t { OFF, ON };

class Controller {
private:
    // 状态数组，直接对应枚举顺序
    State_e states[static_cast<int>(Device_e::COUNT)] = { State_e::OFF };

    // 禁用拷贝构造和赋值运算符,私有构造函数，防止外部实例化
    Controller(const Controller &);
    Controller &operator=(const Controller &);
    Controller() = default;

public:
    // 单例获取方法
    static Controller &instance()
    {
        static Controller instance;
        return instance;
    }

    // 初始化：关闭所有设备
    void init();

    // 设置设备状态
    void set(Device_e _dev, State_e _state)
    {
        const int idx = static_cast<int>(_dev);
        if (idx < 0 || idx >= static_cast<int>(Device_e::COUNT) ||
            states[idx] == _state)
            return;

        // 根据设备和目标状态执行对应宏
        switch (_dev) {
        case Device_e::PUMP_1:
            _state == State_e::ON ? PUMP_1_ON : PUMP_1_OFF; // NOLINT
            break;
        case Device_e::PUMP_2:
            _state == State_e::ON ? PUMP_2_ON : PUMP_2_OFF; // NOLINT
            break;
        case Device_e::VALVE_1:
            _state == State_e::ON ? VALVE_1_ON : VALVE_1_OFF; // NOLINT
            break;
        case Device_e::VALVE_2:
            _state == State_e::ON ? VALVE_2_ON : VALVE_2_OFF; // NOLINT
            break;
        default:
            return;
        }
        states[idx] = _state; // 更新状态记录
    }

    // 获取当前状态
    State_e get(Device_e _dev) const
    {
        const int idx = static_cast<int>(_dev);
        return (idx >= 0 && idx < static_cast<int>(Device_e::COUNT)) ?
                       states[idx] :
                       State_e::OFF;
    }

    // 切换状态（复用set方法）
    void toggle(Device_e _dev)
    {
        set(_dev, get(_dev) == State_e::ON ? State_e::OFF : State_e::ON);
    }

    // 批量应用配置
    void apply(const State_e _config[static_cast<int>(Device_e::COUNT)])
    {
        for (int i = 0; i < static_cast<int>(Device_e::COUNT); i++) {
            set(static_cast<Device_e>(i), _config[i]);
        }
    }
};

// 预设配置
namespace CONFIGS {
constexpr State_e SILVER_PUMP_TEST[] = {
    State_e::ON,  // PUMP_1
    State_e::OFF, // PUMP_2
    State_e::ON,  // VALVE_1
    State_e::ON   // VALVE_2
};

constexpr State_e SILVER_LEFT_PUMP[7][4] = {
    { State_e::OFF, State_e::OFF, State_e::OFF, State_e::OFF },
    { State_e::ON, State_e::ON, State_e::ON, State_e::ON },
    { State_e::ON, State_e::ON, State_e::ON, State_e::ON },
    { State_e::ON, State_e::ON, State_e::ON, State_e::ON },
    { State_e::ON, State_e::ON, State_e::ON, State_e::ON },
    { State_e::ON, State_e::ON, State_e::ON, State_e::ON },
    { State_e::ON, State_e::ON, State_e::ON, State_e::ON }
};

constexpr State_e ALL_ON[] = { State_e::ON, State_e::ON, State_e::ON,
                               State_e::ON };

constexpr State_e ALL_OFF[] = { State_e::OFF, State_e::OFF, State_e::OFF,
                                State_e::OFF };
} // namespace CONFIGS
} // namespace PUMP
