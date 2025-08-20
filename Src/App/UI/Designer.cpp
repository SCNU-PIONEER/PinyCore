#include "Client.hpp"
#include <cstdint>
#include <cstring>
#include "./Designer.hpp"

using namespace UI;

// NOLINTBEGIN

// 兼容RM_UI_Designer
#define rx           end_x
#define ry           end_y
#define font_size    start_angle
#define str_length   end_angle

#define ui_1_frame_t UI::OneGraphicData_s
#define ui_2_frame_t UI::TwoGraphicData_s
#define ui_5_frame_t UI::FiveGraphicData_s
#define ui_7_frame_t UI::SevenGraphicData_s
#define ui_proc_2_frame
#define ui_proc_string_frame
#define SEND_MESSAGE

UI::Info_s newConfig(GraphicData_s *_data)
{
    return UI::Info_s{ .sentState = SendState_e::NOT_SENT,
                       .updateTick = 0,
                       .priorityValue = 0,
                       .config = {
                               .uiType =
                                       static_cast<Type_e>(_data->figure_type),
                               .operateType = static_cast<OperateType_e>(
                                       _data->operate_type),
                               .layer = _data->layer,
                               .color = static_cast<Color_e>(_data->color),
                               .startX = _data->start_x,
                               .startY = _data->start_y,
                               .endX = _data->end_x,
                               .endY = _data->end_y,
                               .radius = _data->radius,
                               .startAngle = _data->start_angle,
                               .endAngle = _data->end_angle,
                               .size = _data->start_angle,
                               /*浮点数：整型数均为 32 位，对于浮点数，实际显示的值为输入的值/100*/
                               .floatNum =
                                       static_cast<float>(_data->radius / 1000),
                               .decimal = _data->end_angle,
                               .intNum = static_cast<int32_t>(_data->radius),
                       } };
}

/* copy zone */
ui_2_frame_t ui_g_dynamic_0;

ui_interface_arc_t *ui_g_dynamic_NewArc =
        (ui_interface_arc_t *)&(ui_g_dynamic_0.data[0]);
ui_interface_number_t *ui_g_dynamic_time =
        (ui_interface_number_t *)&(ui_g_dynamic_0.data[1]);

void _ui_init_g_dynamic_0()
{
    for (int i = 0; i < 2; i++) {
        ui_g_dynamic_0.data[i].figure_name[0] = 0;
        ui_g_dynamic_0.data[i].figure_name[1] = 0;
        ui_g_dynamic_0.data[i].figure_name[2] = i + 0;
        ui_g_dynamic_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_g_dynamic_0.data[i].operate_type = 0;
    }

    ui_g_dynamic_NewArc->figure_type = 4;
    ui_g_dynamic_NewArc->operate_type = 1;
    ui_g_dynamic_NewArc->layer = 0;
    ui_g_dynamic_NewArc->color = 2;
    ui_g_dynamic_NewArc->start_x = 928;
    ui_g_dynamic_NewArc->start_y = 535;
    ui_g_dynamic_NewArc->width = 8;
    ui_g_dynamic_NewArc->start_angle = 50;
    ui_g_dynamic_NewArc->end_angle = 130;
    ui_g_dynamic_NewArc->rx = 425;
    ui_g_dynamic_NewArc->ry = 410;

    ui_g_dynamic_time->figure_type = 5;
    ui_g_dynamic_time->operate_type = 1;
    ui_g_dynamic_time->layer = 0;
    ui_g_dynamic_time->color = 2;
    ui_g_dynamic_time->start_x = 893;
    ui_g_dynamic_time->start_y = 899;
    ui_g_dynamic_time->width = 2;
    ui_g_dynamic_time->font_size = 20;
    ui_g_dynamic_time->radius = 0;

    ui_proc_2_frame(&ui_g_dynamic_0);
    SEND_MESSAGE((uint8_t *)&ui_g_dynamic_0, sizeof(ui_g_dynamic_0));
}

void _ui_update_g_dynamic_0()
{
    for (int i = 0; i < 2; i++) {
        ui_g_dynamic_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_g_dynamic_0);
    SEND_MESSAGE((uint8_t *)&ui_g_dynamic_0, sizeof(ui_g_dynamic_0));
}

void _ui_remove_g_dynamic_0()
{
    for (int i = 0; i < 2; i++) {
        ui_g_dynamic_0.data[i].operate_type = 3;
    }
}

ui_string_frame_t ui_g_dynamic_1;
ui_interface_string_t *ui_g_dynamic_chassis_state = &(ui_g_dynamic_1.option);

void _ui_init_g_dynamic_1()
{
    ui_g_dynamic_1.option.figure_name[0] = 0;
    ui_g_dynamic_1.option.figure_name[1] = 0;
    ui_g_dynamic_1.option.figure_name[2] = 3;
    ui_g_dynamic_1.option.operate_type = 1;

    ui_g_dynamic_chassis_state->figure_type = 7;
    ui_g_dynamic_chassis_state->operate_type = 1;
    ui_g_dynamic_chassis_state->layer = 0;
    ui_g_dynamic_chassis_state->color = 2;
    ui_g_dynamic_chassis_state->start_x = 169;
    ui_g_dynamic_chassis_state->start_y = 492;
    ui_g_dynamic_chassis_state->width = 2;
    ui_g_dynamic_chassis_state->font_size = 20;
    ui_g_dynamic_chassis_state->str_length = 6;
    strcpy(ui_g_dynamic_1.string, "NORMAL");


    ui_proc_string_frame(&ui_g_dynamic_1);
    SEND_MESSAGE((uint8_t *)&ui_g_dynamic_1, sizeof(ui_g_dynamic_1));
}

void _ui_update_g_dynamic_1()
{
    ui_g_dynamic_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_dynamic_1);
    SEND_MESSAGE((uint8_t *)&ui_g_dynamic_1, sizeof(ui_g_dynamic_1));
}

void _ui_remove_g_dynamic_1()
{
    ui_g_dynamic_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_dynamic_1);
    SEND_MESSAGE((uint8_t *)&ui_g_dynamic_1, sizeof(ui_g_dynamic_1));
}

void ui_init_g_dynamic()
{
    _ui_init_g_dynamic_0();
    _ui_init_g_dynamic_1();
}

void ui_update_g_dynamic()
{
    _ui_update_g_dynamic_0();
    _ui_update_g_dynamic_1();
}

void ui_remove_g_dynamic()
{
    _ui_remove_g_dynamic_0();
    _ui_remove_g_dynamic_1();
}

ui_string_frame_t ui_g_static_0;
ui_interface_string_t *ui_g_static_chassis = &(ui_g_static_0.option);

void _ui_init_g_static_0()
{
    ui_g_static_0.option.figure_name[0] = 0;
    ui_g_static_0.option.figure_name[1] = 1;
    ui_g_static_0.option.figure_name[2] = 1;
    ui_g_static_0.option.operate_type = 1;

    ui_g_static_chassis->figure_type = 7;
    ui_g_static_chassis->operate_type = 1;
    ui_g_static_chassis->layer = 1;
    ui_g_static_chassis->color = 2;
    ui_g_static_chassis->start_x = 54;
    ui_g_static_chassis->start_y = 479;
    ui_g_static_chassis->width = 1;
    ui_g_static_chassis->font_size = 14;
    ui_g_static_chassis->str_length = 7;
    strcpy(ui_g_static_0.string, "Chassis");


    ui_proc_string_frame(&ui_g_static_0);
    SEND_MESSAGE((uint8_t *)&ui_g_static_0, sizeof(ui_g_static_0));
}

void _ui_update_g_static_0()
{
    ui_g_static_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_static_0);
    SEND_MESSAGE((uint8_t *)&ui_g_static_0, sizeof(ui_g_static_0));
}

void _ui_remove_g_static_0()
{
    ui_g_static_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_static_0);
    SEND_MESSAGE((uint8_t *)&ui_g_static_0, sizeof(ui_g_static_0));
}

void ui_init_g_static() { _ui_init_g_static_0(); }

void ui_update_g_static() { _ui_update_g_static_0(); }

void ui_remove_g_static() { _ui_remove_g_static_0(); }

// NOLINTEND
