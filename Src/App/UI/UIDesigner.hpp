#pragma once

#include "./UIClient.hpp"
#include <cstdint>
/* need to config */
static constexpr uint8_t DYNAIMIC_NUM = 3;
static constexpr uint8_t STATIC_NUM = 1;

void dynamicConfig(UI::Info_s *_info);
void staticConfig(UI::Info_s *_info);

namespace UI::DYNAMIC_UI {

enum class Name_e : uint8_t {
    CHASSIS_STATE,
    GIMBAL_STATE,
    ARM_STATE,
};

}
