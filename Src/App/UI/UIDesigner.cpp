#include "UIProtocol.hpp"
#include <cstring>
#include "./UIDesigner.hpp"

using namespace UI;

// NOLINTBEGIN
// 兼容RM_UI_Designer

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

void dynamicConfig(UI::Info_s *_info)
{
    // _info[0].config = {
    //     .priority = UI::Priority_e::HIGH,
    //     .name = "01",
    //     .uiType = Type_e::INT,
    //     .operateType = OperateType_e::MODIFY,
    //     .layer = 1,
    //     .color = Color_e::RED_BLUE,
    //     .width = 1,
    //     .startX = 700,
    //     .startY = 840,
    //     //直线，矩形，正圆，圆弧需要使用
    //     .endX = 600,
    //     .endY = 600,
    //     //正圆半径
    //     .radius = 100,
    //     //圆弧起始角度，终止角度
    //     .startAngle = 20,
    //     .endAngle = 300,
    //     //浮点数：整型数均为 32 位，对于浮点数，实际显示的值为输入的值/1000
    //     .floatNum = 100.f,
    //     .decimal = 0,
    //     //整型数数据
    //     .intNum = 1000,
    //     //字体大小
    //     .size = 11,
    //     .text = "1234567890",
    // };
    using namespace DYNAMIC_UI;
    _info[(uint8_t)Name_e::CHASSIS_STATE].config = {
        .priority = UI::Priority_e::LOW,
        .name = "01",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::YELLOW,
        .width = 3,
        .startX = 200,
        .startY = 850,
        .size = 16,
        .text = "STOP",
    };

    _info[(uint8_t)Name_e::GIMBAL_STATE].config = {
        .priority = UI::Priority_e::LOW,
        .name = "02",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::YELLOW,
        .width = 3,
        .startX = 180,
        .startY = 800,
        .size = 16,
        .text = "STOP",
    };

    _info[(uint8_t)Name_e::ARM_STATE].config = {
        .priority = UI::Priority_e::LOW,
        .name = "03",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::YELLOW,
        .width = 3,
        .startX = 130,
        .startY = 750,
        .size = 16,
        .text = "STOP",
    };

    _info[(uint8_t)Name_e::ARM_CLAMP_STATE].config = {
        .priority = UI::Priority_e::MID,
        .name = "04",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::YELLOW,
        .width = 3,
        .startX = 225,
        .startY = 700,
        .size = 16,
        .text = "NONE",
    };

    _info[(uint8_t)Name_e::ARM_CUSTOM_CTRL_STATE].config = {
        .priority = UI::Priority_e::MID,
        .name = "05",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::YELLOW,
        .width = 3,
        .startX = 250,
        .startY = 650,
        .size = 16,
        .text = "OFFLINE",
    };
};

void staticConfig(UI::Info_s *_info)
{
    using namespace STATIC_UI;
    _info[(uint8_t)Name_e::CHASSIS_STATE].config = {
        .name = "1",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::GREEN,
        .width = 3,
        .startX = 76,
        .startY = 850,
        .size = 16,
        .text = "CHASSIS",
    };

    _info[(uint8_t)Name_e::GIMBAL_STATE].config = {
        .name = "2",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::GREEN,
        .width = 3,
        .startX = 76,
        .startY = 800,
        .size = 16,
        .text = "GIMBAL",
    };

    _info[(uint8_t)Name_e::ARM_STATE].config = {
        .name = "3",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::GREEN,
        .width = 3,
        .startX = 76,
        .startY = 750,
        .size = 16,
        .text = "ARM",
    };

    _info[(uint8_t)Name_e::ARM_CLAMP_STATE].config = {
        .name = "4",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::GREEN,
        .width = 3,
        .startX = 76,
        .startY = 700,
        .size = 16,
        .text = "ARM_CLAMP",
    };

    _info[(uint8_t)Name_e::ARM_CUSTOM_CTRL_STATE].config = {
        .name = "5",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::GREEN,
        .width = 3,
        .startX = 76,
        .startY = 650,
        .size = 16,
        .text = "ARM_CUSTOM",
    };
}

#pragma GCC diagnostic pop
