#pragma once

#include "Rc.hpp"
#include <cstdint>

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
enum class PumpDevice_e : uint8_t { PUMP_1, PUMP_2, VALVE_1, VALVE_2, COUNT };

enum class PumpState_e : uint8_t { OFF, ON };

class PumpController {
private:
    // 状态数组，直接对应枚举顺序
    PumpState_e states[static_cast<int>(PumpDevice_e::COUNT)] = {
        PumpState_e::OFF
    };

    // 私有构造函数，防止外部实例化
    // 禁用拷贝构造和赋值运算符
    PumpController(const PumpController &);
    PumpController &operator=(const PumpController &);
    PumpController() = default;

public:
    // 单例获取方法（放在类内部作为静态成员）
    static PumpController &instance()
    {
        static PumpController instance;
        return instance;
    }

    // 初始化：关闭所有设备
    void init(); // 仅声明，实现移至cpp文件

    // 设置设备状态
    void set(PumpDevice_e _dev, PumpState_e _state)
    {
        const int idx = static_cast<int>(_dev);
        if (idx < 0 || idx >= static_cast<int>(PumpDevice_e::COUNT))
            return;

        // 状态不变则不操作硬件
        if (states[idx] == _state)
            return;

        // 根据设备和目标状态执行对应宏
        switch (_dev) {
        case PumpDevice_e::PUMP_1:
            _state == PumpState_e::ON ? PUMP_1_ON : PUMP_1_OFF; // NOLINT
            break;
        case PumpDevice_e::PUMP_2:
            _state == PumpState_e::ON ? PUMP_2_ON : PUMP_2_OFF; // NOLINT
            break;
        case PumpDevice_e::VALVE_1:
            _state == PumpState_e::ON ? VALVE_1_ON : VALVE_1_OFF; // NOLINT
            break;
        case PumpDevice_e::VALVE_2:
            _state == PumpState_e::ON ? VALVE_2_ON : VALVE_2_OFF; // NOLINT
            break;
        default:
            return;
        }
        states[idx] = _state; // 更新状态记录
    }

    // 获取当前状态
    PumpState_e get(PumpDevice_e _dev) const
    {
        const int idx = static_cast<int>(_dev);
        return (idx >= 0 && idx < static_cast<int>(PumpDevice_e::COUNT)) ?
                       states[idx] :
                       PumpState_e::OFF;
    }

    // 切换状态（复用set方法）
    void toggle(PumpDevice_e _dev)
    {
        set(_dev,
            get(_dev) == PumpState_e::ON ? PumpState_e::OFF : PumpState_e::ON);
    }

    // 批量应用配置
    void apply(const PumpState_e _config[static_cast<int>(PumpDevice_e::COUNT)])
    {
        for (int i = 0; i < static_cast<int>(PumpDevice_e::COUNT); i++) {
            set(static_cast<PumpDevice_e>(i), _config[i]);
        }
    }
};

// 预设配置
namespace PUMPCONFIGS {
constexpr PumpState_e SILVER_PUMP_TEST[] = {
    PumpState_e::ON,  // PUMP_1
    PumpState_e::OFF, // PUMP_2
    PumpState_e::ON,  // VALVE_1
    PumpState_e::ON   // VALVE_2
};

constexpr PumpState_e SILVER_LEFT_PUMP[7][4] = {
    { PumpState_e::OFF, PumpState_e::OFF, PumpState_e::OFF, PumpState_e::OFF },
    { PumpState_e::ON, PumpState_e::ON, PumpState_e::ON, PumpState_e::ON },
    { PumpState_e::ON, PumpState_e::ON, PumpState_e::ON, PumpState_e::ON },
    { PumpState_e::ON, PumpState_e::ON, PumpState_e::ON, PumpState_e::ON },
    { PumpState_e::ON, PumpState_e::ON, PumpState_e::ON, PumpState_e::ON },
    { PumpState_e::ON, PumpState_e::ON, PumpState_e::ON, PumpState_e::ON },
    { PumpState_e::ON, PumpState_e::ON, PumpState_e::ON, PumpState_e::ON }
};

constexpr PumpState_e ALL_PUMP_ON[] = { PumpState_e::ON, PumpState_e::ON,
                                        PumpState_e::ON, PumpState_e::ON };

constexpr PumpState_e ALL_PUMP_OFF[] = { PumpState_e::OFF, PumpState_e::OFF,
                                         PumpState_e::OFF, PumpState_e::OFF };
} // namespace PUMPCONFIGS
