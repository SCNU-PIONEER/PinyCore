/**
 * @file UIBuilder.hpp
 * @brief 流式构建UI图形
 *
 * This file contains code from Priority_UI
 * Copyright (c) 2025 IsaacZH
 * Used under MIT License - https://opensource.org/licenses/MIT
 */

#pragma once

#include <cstdint>
#include "./Protocol.hpp"

namespace UI {

class GraphicBuilder {
public:
    GraphicBuilder &create(const char *_name, OperateType_e _operateType,
                           uint8_t _layer);

    GraphicBuilder &color(Color_e _color);

    GraphicBuilder &width(uint16_t _width);

    GraphicBuilder &line(uint16_t _startX, uint16_t _startY, uint16_t _endX,
                         uint16_t _endY);

    GraphicBuilder &rectangle(uint16_t _startX, uint16_t _startY,
                              uint16_t _endX, uint16_t _endY);

    GraphicBuilder &circle(uint16_t _centerX, uint16_t _centerY,
                           uint16_t _radius);

    GraphicBuilder &ellipse(uint16_t _centerX, uint16_t _centerY,
                            uint16_t _xRadius, uint16_t _yRadius);

    GraphicBuilder &arc(uint16_t _startAngle, uint16_t _endAngle,
                        uint16_t _centerX, uint16_t _centerY, uint16_t _xRadius,
                        uint16_t _yRadius);

    GraphicBuilder &floatNum(uint16_t _fontSize, uint16_t _decimal,
                             uint16_t _startX, uint16_t _startY, int32_t _num);

    GraphicBuilder &intNum(uint16_t _startX, uint16_t _startY,
                           uint8_t _fontSize, int32_t _num);

    GraphicBuilder &character(uint8_t _fontSize, uint16_t _length,
                              uint16_t _startX, uint16_t _startY);

    GraphicData_s build();

private:
    GraphicData_s data_{};
    bool initialized_ = false;
};

} // namespace UI
