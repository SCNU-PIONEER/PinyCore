#include "Pump.hpp"
#include "main.h"
#include "StmLog.hpp"

using namespace PUMP;

// 实现初始化函数
Controller::Controller()
{
    apply(PUMP::ALL_OFF);
    LOG::info("Pump", "initialized");
}

void Controller::set(Device_e _dev, State_e _state)
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

State_e Controller::get(Device_e _dev) const
{
    const int idx = static_cast<int>(_dev);
    return (idx >= 0 && idx < static_cast<int>(Device_e::COUNT)) ? states[idx] :
                                                                   State_e::OFF;
}


// 切换状态（复用set方法）
void Controller::toggle(Device_e _dev)
{
    set(_dev, get(_dev) == State_e::ON ? State_e::OFF : State_e::ON);
}

// 批量应用配置
void Controller::apply(const State_e _config[static_cast<int>(Device_e::COUNT)])
{
    for (int i = 0; i < static_cast<int>(Device_e::COUNT); i++) {
        set(static_cast<Device_e>(i), _config[i]);
    }
}
