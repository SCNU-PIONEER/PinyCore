#pragma once

#include "./UIProtocol.hpp"
#include "./UIClient.hpp"
#include <cstdint>

#define ui_string_frame_t      UI::CharGraphicData_s
#define ui_string_frame_t      UI::CharGraphicData_s
#define ui_interface_rect_t    UI::GraphicData_s
#define ui_interface_line_t    UI::GraphicData_s
#define ui_interface_round_t   UI::GraphicData_s
#define ui_interface_ellipse_t UI::GraphicData_s
#define ui_interface_arc_t     UI::GraphicData_s
#define ui_interface_number_t  UI::GraphicData_s
#define ui_interface_string_t  UI::GraphicData_s

// NOLINTBEGIN

/* need to config */
static constexpr uint8_t UIdynamicNum = 3;
static constexpr uint8_t UIconstNum = 1;

/* don't change */
UI::Info_s newConfig(UI::GraphicData_s *_data);

/* copy zone */
extern ui_interface_arc_t *ui_g_dynamic_NewArc;
extern ui_interface_number_t *ui_g_dynamic_time;
extern ui_interface_string_t *ui_g_dynamic_chassis_state;

void ui_init_g_dynamic();
void ui_update_g_dynamic();
void ui_remove_g_dynamic();

extern ui_interface_string_t *ui_g_static_chassis;

void ui_init_g_static();
void ui_update_g_static();
void ui_remove_g_static();

// NOLINTEND
