/**
 * @file UIBuilder.hpp
 * @brief 流式构建UI图形
 *
 * This file contains code from Priority_UI
 * Copyright (c) 2025 IsaacZH
 * Used under MIT License - https://opensource.org/licenses/MIT
 */

#include "./Builder.hpp"
#include <cstdint>
#include <cstring>

using namespace UI;

GraphicBuilder &GraphicBuilder::create(const char *_name,
                                       OperateType_e _operateType,
                                       uint8_t _layer)
{
    std::memset(&data_, 0, sizeof(GraphicData_s));

    std::strncpy((char *)data_.figure_name, _name, 3);
    data_.operate_type = _operateType;
    data_.layer = _layer;
    initialized_ = true;
    return *this;
}

GraphicBuilder &GraphicBuilder::color(Color_e _color)
{
    if (!initialized_)
        return *this;
    data_.color = static_cast<uint8_t>(_color);
    return *this;
}

GraphicBuilder &GraphicBuilder::width(uint16_t _width)
{
    if (!initialized_)
        return *this;
    data_.width = _width;
    return *this;
}

GraphicBuilder &GraphicBuilder::line(uint16_t _start_x, uint16_t _start_y,
                                     uint16_t _end_x, uint16_t _end_y)
{
    if (!initialized_)
        return *this;

    data_.figure_type = static_cast<uint32_t>(GraphicType_e::LINE);
    data_.start_x = _start_x;
    data_.start_y = _start_y;
    data_.end_x = _end_x;
    data_.end_y = _end_y;
    return *this;
}

GraphicBuilder &GraphicBuilder::rectangle(uint16_t _start_x, uint16_t _start_y,
                                          uint16_t _end_x, uint16_t _end_y)
{
    if (!initialized_)
        return *this;

    data_.figure_type = static_cast<uint32_t>(GraphicType_e::RECTANGLE);
    data_.start_x = _start_x;
    data_.start_y = _start_y;
    data_.end_x = _end_x;
    data_.end_y = _end_y;
    return *this;
}

GraphicBuilder &GraphicBuilder::circle(uint16_t _centerX, uint16_t _centerY,
                                       uint16_t _radius)
{
    if (!initialized_)
        return *this;

    data_.figure_type = static_cast<uint32_t>(GraphicType_e::CIRCLE);
    data_.start_x = _centerX;
    data_.start_y = _centerY;
    data_.radius = _radius;
    return *this;
}

GraphicBuilder &GraphicBuilder::ellipse(uint16_t _centerX, uint16_t _centerY,
                                        uint16_t _xRadius, uint16_t _yRadius)
{
    if (!initialized_)
        return *this;

    data_.figure_type = static_cast<uint32_t>(GraphicType_e::ELLIPSE);
    data_.start_x = _centerX;
    data_.start_y = _centerY;
    data_.end_x = _xRadius; // 用end_x存储x半径
    data_.end_y = _yRadius; // 用end_y存储y半径
    return *this;
}

GraphicBuilder &GraphicBuilder::arc(uint16_t _start_angle, uint16_t _end_angle,
                                    uint16_t _centerX, uint16_t _centerY,
                                    uint16_t _xRadius, uint16_t _yRadius)
{
    if (!initialized_)
        return *this;

    data_.figure_type = static_cast<uint32_t>(GraphicType_e::ARC);
    data_.start_angle = _start_angle;
    data_.end_angle = _end_angle;
    data_.start_x = _centerX;
    data_.start_y = _centerY;
    data_.end_x = _xRadius;
    data_.end_y = _yRadius;
    return *this;
}

GraphicBuilder &GraphicBuilder::floatNum(uint16_t _fontSize, uint16_t _decimal,
                                         uint16_t _start_x, uint16_t _start_y,
                                         int32_t _num)
{
    if (!initialized_)
        return *this;

    data_.figure_type = static_cast<uint32_t>(GraphicType_e::FLOAT);
    data_.start_angle = _fontSize;
    data_.end_angle = _decimal;
    data_.start_x = _start_x;
    data_.start_y = _start_y;
    data_.radius = _num;
    data_.end_x = _num >> 10;
    data_.end_x = _num >> 21;
    return *this;
}

GraphicBuilder &GraphicBuilder::intNum(uint16_t _start_x, uint16_t _start_y,
                                       uint8_t _fontSize, int32_t _num)
{
    if (!initialized_)
        return *this;

    data_.figure_type = static_cast<uint32_t>(GraphicType_e::INT);
    data_.start_angle = _fontSize;
    data_.end_angle = 0;
    data_.start_x = _start_x;
    data_.start_y = _start_y;
    data_.radius = _num;
    data_.end_x = _num >> 10;
    data_.end_y = _num >> 21;
    return *this;
}

GraphicBuilder &GraphicBuilder::character(uint8_t _fontSize, uint16_t _length,
                                          uint16_t _start_x, uint16_t _start_y)
{
    if (!initialized_)
        return *this;

    data_.figure_type = static_cast<uint32_t>(GraphicType_e::CHAR);
    data_.start_angle = _fontSize;
    data_.end_angle = _length;
    data_.start_x = _start_x;
    data_.start_y = _start_y;
    return *this;
}

GraphicData_s GraphicBuilder::build()
{
    if (!initialized_) {
        std::memset(&data_, 0, sizeof(GraphicData_s));
    }
    return data_;
}
