/**
 * @file UIBuilder.hpp
 * @brief 流式构建UI图形
 *
 * This file contains code from Priority_UI
 * Copyright (c) 2025 IsaacZH
 * Used under MIT License - https://opensource.org/licenses/MIT
 */

#include "UIBuilder.hpp"
#include <cstring>

using namespace UI;

GraphicBuilder &GraphicBuilder::create(const char *_name,
                                       OperateType_e _operateType,
                                       uint8_t _layer)
{
    std::memset(&data_, 0, sizeof(GraphicData_s));

    std::strncpy((char *)data_.graphicName, _name, 3);
    data_.operateTpye = _operateType;
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

GraphicBuilder &GraphicBuilder::line(uint16_t _startX, uint16_t _startY,
                                     uint16_t _endX, uint16_t _endY)
{
    if (!initialized_)
        return *this;

    data_.graphicTpye = GraphicType_e::LINE;
    data_.startX = _startX;
    data_.startY = _startY;
    data_.endX = _endX;
    data_.endY = _endY;
    return *this;
}

GraphicBuilder &GraphicBuilder::rectangle(uint16_t _startX, uint16_t _startY,
                                          uint16_t _endX, uint16_t _endY)
{
    if (!initialized_)
        return *this;

    data_.graphicTpye = GraphicType_e::RECTANGLE;
    data_.startX = _startX;
    data_.startY = _startY;
    data_.endX = _endX;
    data_.endY = _endY;
    return *this;
}

GraphicBuilder &GraphicBuilder::circle(uint16_t _centerX, uint16_t _centerY,
                                       uint16_t _radius)
{
    if (!initialized_)
        return *this;

    data_.graphicTpye = GraphicType_e::CIRCLE;
    data_.startX = _centerX;
    data_.startY = _centerY;
    data_.radius = _radius;
    return *this;
}

GraphicBuilder &GraphicBuilder::ellipse(uint16_t _centerX, uint16_t _centerY,
                                        uint16_t _xRadius, uint16_t _yRadius)
{
    if (!initialized_)
        return *this;

    data_.graphicTpye = GraphicType_e::ELLIPSE;
    data_.startX = _centerX;
    data_.startY = _centerY;
    data_.endX = _xRadius; // 用end_x存储x半径
    data_.endY = _yRadius; // 用end_y存储y半径
    return *this;
}

GraphicBuilder &GraphicBuilder::arc(uint16_t _startAngle, uint16_t _endAngle,
                                    uint16_t _centerX, uint16_t _centerY,
                                    uint16_t _xRadius, uint16_t _yRadius)
{
    if (!initialized_)
        return *this;

    data_.graphicTpye = GraphicType_e::ARC;
    data_.startAngle = _startAngle;
    data_.endAngle = _endAngle;
    data_.startX = _centerX;
    data_.startY = _centerY;
    data_.endX = _xRadius;
    data_.endY = _yRadius;
    return *this;
}

GraphicBuilder &GraphicBuilder::floatNum(uint16_t _fontSize, uint16_t _decimal,
                                         uint16_t _startX, uint16_t _startY,
                                         int32_t _num)
{
    if (!initialized_)
        return *this;

    data_.graphicTpye = GraphicType_e::FLOAT; // 浮点数类型
    data_.startAngle = _fontSize;
    data_.endAngle = _decimal;
    data_.startX = _startX;
    data_.startY = _startY;
    data_.radius = _num;
    data_.endX = _num >> 10;
    data_.endX = _num >> 21;
    return *this;
}

GraphicBuilder &GraphicBuilder::intNum(uint16_t _startX, uint16_t _startY,
                                       uint8_t _fontSize, int32_t _num)
{
    if (!initialized_)
        return *this;

    data_.graphicTpye = GraphicType_e::INT;
    data_.startAngle = _fontSize;
    data_.endAngle = 0;
    data_.startX = _startX;
    data_.startY = _startY;
    data_.radius = _num;
    data_.endX = _num >> 10;
    data_.endY = _num >> 21;
    return *this;
}

GraphicBuilder &GraphicBuilder::character(uint8_t _fontSize, uint16_t _length,
                                          uint16_t _startX, uint16_t _startY)
{
    if (!initialized_)
        return *this;

    data_.graphicTpye = GraphicType_e::CHAR;
    data_.startAngle = _fontSize;
    data_.endAngle = _length;
    data_.startX = _startX;
    data_.startY = _startY;
    return *this;
}

GraphicData_s GraphicBuilder::build()
{
    if (!initialized_) {
        std::memset(&data_, 0, sizeof(GraphicData_s));
    }
    return data_;
}
