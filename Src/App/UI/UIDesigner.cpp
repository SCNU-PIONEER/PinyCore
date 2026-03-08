#include "UIProtocol.hpp"
#include <cstring>
#include "./UIDesigner.hpp"

using namespace UI;

// NOLINTBEGIN
// 兼容RM_UI_Designer

void dynamicConfig(UI::Info_s *_info)
{
    _info[0].config = {
        .priority = UI::Priority_e::HIGH,
        .name = "01",
        .uiType = Type_e::INT,
        .operateType = OperateType_e::MODIFY,
        .layer = 1,
        .color = Color_e::RED_BLUE,
        .width = 1,
        .startX = 700,
        .startY = 840,
        //直线，矩形，正圆，圆弧需要使用
        .endX = 600,
        .endY = 600,
        //正圆半径
        .radius = 100,
        //圆弧起始角度，终止角度
        .startAngle = 20,
        .endAngle = 300,
        //浮点数：整型数均为 32 位，对于浮点数，实际显示的值为输入的值/1000
        .floatNum = 100.f,
        .decimal = 0,
        //整型数数据
        .intNum = 1000,
        //字体大小
        .size = 11,
        .text = "1234567890",
    };
};

void staticConfig(UI::Info_s *_info)
{
    _info[0].config = {
        .priority = UI::Priority_e::HIGH,
        .name = "ne",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::ADD,
        .layer = 0,
        .color = Color_e::RED_BLUE,
        .width = 2,
        .startX = 76,
        .startY = 840,
        //直线，矩形，正圆，圆弧需要使用
        .endX = 600,
        .endY = 600,
        //正圆半径
        .radius = 100,
        //圆弧起始角度，终止角度
        .startAngle = 20,
        .endAngle = 300,
        //浮点数：整型数均为 32 位，对于浮点数，实际显示的值为输入的值/1000
        .floatNum = 100.f,
        .decimal = 0,
        //整型数数据
        .intNum = 1000,
        //字体大小
        .size = 11,
        .text = "1234567890",
    };
}