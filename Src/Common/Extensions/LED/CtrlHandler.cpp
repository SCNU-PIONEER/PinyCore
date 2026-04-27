#include "LEDManager.hpp"

#include <array>

using namespace LED;

namespace {

struct CtrlRange_s {
    uint8_t len = 0;
    RGB_s *ptr = nullptr;
};

bool getCtrlRange(std::vector<RGB_s> &_ledColors, uint8_t _index, uint8_t _ctrlNum, CtrlRange_s &_range)
{
    if (_ctrlNum == 0 || _index >= _ledColors.size()) {
        return false;
    }

    const size_t available = _ledColors.size() - _index;
    const uint8_t available8 = (available > 0xFFU) ? 0xFFU : static_cast<uint8_t>(available);
    _range.len = (_ctrlNum < available8) ? _ctrlNum : available8;
    _range.ptr = &_ledColors[_index];
    return _range.len > 0;
}

RGB_s rainbowWheel(uint8_t _position)
{
    _position = 255 - _position;
    if (_position < 85) {
        return { static_cast<uint8_t>(255 - (_position * 3)), 0, static_cast<uint8_t>(_position * 3) };
    }
    if (_position < 170) {
        _position -= 85;
        return { 0, static_cast<uint8_t>(_position * 3), static_cast<uint8_t>(255 - (_position * 3)) };
    }

    _position -= 170;
    return { static_cast<uint8_t>(_position * 3), static_cast<uint8_t>(255 - (_position * 3)), 0 };
}

const RGB_s &rainbowLut(uint8_t _position)
{
    static const std::array<RGB_s, 256> LUT = [] {
        std::array<RGB_s, 256> table{};
        for (int i = 0; i < 256; ++i) {
            table[static_cast<size_t>(i)] = rainbowWheel(static_cast<uint8_t>(i));
        }
        return table;
    }();

    return LUT[_position];
}

inline RGB_s scaleColor8(const RGB_s &_color, uint8_t _brightness)
{
    const uint8_t r = static_cast<uint8_t>((static_cast<uint16_t>(_color.rgb.r) * _brightness) >> 8);
    const uint8_t g = static_cast<uint8_t>((static_cast<uint16_t>(_color.rgb.g) * _brightness) >> 8);
    const uint8_t b = static_cast<uint8_t>((static_cast<uint16_t>(_color.rgb.b) * _brightness) >> 8);
    return { r, g, b };
}

void fillRange(const CtrlRange_s &_range, const RGB_s &_color)
{
    RGB_s *start = _range.ptr;
    for (uint8_t i = 0; i < _range.len; ++i) {
        start[i] = _color;
    }
}

void fillRangeColorCode(const CtrlRange_s &_range, RGB_s::Color_e _color)
{
    RGB_s *start = _range.ptr;
    for (uint8_t i = 0; i < _range.len; ++i) {
        start[i].setColorCode(_color);
    }
}

} // namespace

void LEDs::handleOff(uint8_t _index, uint8_t _ctrlNum)
{
    CtrlRange_s range;
    if (getCtrlRange(ledColors_, _index, _ctrlNum, range)) {
        fillRange(range, { 0, 0, 0 });
    }
    this->show();
}

void LEDs::handleOnInNormal(uint8_t _index, uint8_t _ctrlNum)
{
    CtrlRange_s range;
    if (getCtrlRange(ledColors_, _index, _ctrlNum, range)) {
        fillRange(range, { 0, 255, 0 });
    }
    this->show();
}

void LEDs::handleOnInWarn(uint8_t _index, uint8_t _ctrlNum)
{
    CtrlRange_s range;
    if (getCtrlRange(ledColors_, _index, _ctrlNum, range)) {
        fillRange(range, { 255, 255, 0 });
    }
    this->show();
}

void LEDs::handleOnInError(uint8_t _index, uint8_t _ctrlNum)
{
    CtrlRange_s range;
    if (getCtrlRange(ledColors_, _index, _ctrlNum, range)) {
        fillRange(range, { 255, 0, 0 });
    }
    this->show();
}

void LEDs::handleBlinkRGB(uint8_t _index, uint8_t _ctrlNum)
{
    static constexpr RGB_s::Color_e RGB[3] = { RGB_s::Color_e::Red, RGB_s::Color_e::Green, RGB_s::Color_e::Blue };
    CtrlRange_s range;
    if (!getCtrlRange(ledColors_, _index, _ctrlNum, range)) {
        return;
    }

    uint8_t cnt = 0;
    uint8_t flowingFlag = 0;
    while (cnt++ < 3) {
        vTaskDelay(500 / portTICK_PERIOD_MS); // interval 500ms
        fillRangeColorCode(range, RGB[flowingFlag]);
        this->show();
        flowingFlag = (flowingFlag + 1) % 3; // 0, 1, 2
    }
}

void LEDs::handleBlinkRed(uint8_t _index, uint8_t _ctrlNum) { handleBlinkColor(_index, _ctrlNum, RGB_s::Color_e::Red); }

void LEDs::handleBlinkGreen(uint8_t _index, uint8_t _ctrlNum)
{
    handleBlinkColor(_index, _ctrlNum, RGB_s::Color_e::Green);
}

void LEDs::handleBlinkBlue(uint8_t _index, uint8_t _ctrlNum)
{
    handleBlinkColor(_index, _ctrlNum, RGB_s::Color_e::Blue);
}

void LEDs::handleBlinkColor(uint8_t _index, uint8_t _ctrlNum, RGB_s::Color_e _color)
{
    CtrlRange_s range;
    if (!getCtrlRange(ledColors_, _index, _ctrlNum, range)) {
        return;
    }

    for (uint8_t cnt = 0; cnt < 3; ++cnt) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        fillRangeColorCode(range, _color);
        this->show();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        fillRange(range, { 0, 0, 0 });
        this->show();
    }
}

void LEDs::handleRainbowFlow(uint8_t _index, uint8_t _ctrlNum)
{
    CtrlRange_s range;
    if (!getCtrlRange(ledColors_, _index, _ctrlNum, range)) {
        return;
    }

    RGB_s *start = range.ptr;
    for (uint8_t i = 0; i < 255; ++i) {
        uint8_t hue = i;
        for (uint8_t j = 0; j < range.len; ++j) {
            start[j] = rainbowLut(hue);
            ++hue;
        }
        this->show();
        vTaskDelay(100 / portTICK_PERIOD_MS); // interval 10ms
    }
}

void LEDs::handleRainbowFlowReverse(uint8_t _index, uint8_t _ctrlNum)
{
    CtrlRange_s range;
    if (!getCtrlRange(ledColors_, _index, _ctrlNum, range)) {
        return;
    }

    RGB_s *start = range.ptr;
    for (int i = 254; i >= 0; --i) {
        uint8_t hue = static_cast<uint8_t>(i);
        for (uint8_t j = 0; j < range.len; ++j) {
            start[j] = rainbowLut(hue);
            ++hue;
        }
        this->show();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void LEDs::handleRainbowFlowSnake(uint8_t _index, uint8_t _ctrlNum)
{
    CtrlRange_s range;
    if (!getCtrlRange(ledColors_, _index, _ctrlNum, range)) {
        return;
    }

    constexpr uint8_t SNAKE_LEN = 8;
    const uint8_t bodyLen = (range.len < SNAKE_LEN) ? range.len : SNAKE_LEN;
    std::array<uint8_t, SNAKE_LEN> brightnessLut{};
    for (uint8_t i = 0; i < bodyLen; ++i) {
        brightnessLut[i] = static_cast<uint8_t>(255 - ((i * 255) / bodyLen));
    }

    fillRange(range, { 0, 0, 0 });
    uint8_t head = 0;
    uint8_t prevTail = 0;
    bool hasPrevTail = false;
    RGB_s *start = range.ptr;

    for (uint8_t step = 0; step < 255; ++step) {
        if (hasPrevTail) {
            start[prevTail] = { 0, 0, 0 };
        }

        uint8_t pos = head;
        for (uint8_t body = 0; body < bodyLen; ++body) {
            const RGB_s &base = rainbowLut(static_cast<uint8_t>((step * 4) + (body * 18)));
            start[pos] = scaleColor8(base, brightnessLut[body]);
            if (body + 1 == bodyLen) {
                prevTail = pos;
                hasPrevTail = true;
            }
            pos = (pos == 0) ? (range.len - 1) : (pos - 1);
        }

        this->show();
        vTaskDelay(20 / portTICK_PERIOD_MS);

        ++head;
        if (head == range.len) {
            head = 0;
        }
    }
}

void LEDs::handleRainbowBreath(uint8_t _index, uint8_t _ctrlNum)
{
    CtrlRange_s range;
    if (!getCtrlRange(ledColors_, _index, _ctrlNum, range)) {
        return;
    }

    RGB_s *start = range.ptr;
    for (uint8_t i = 0; i < 255; ++i) {
        const uint8_t brightness = (i < 128) ? static_cast<uint8_t>(i * 2) : static_cast<uint8_t>((255 - i) * 2);
        uint8_t hue = i;
        for (uint8_t j = 0; j < range.len; ++j) {
            start[j] = scaleColor8(rainbowLut(hue), brightness);
            ++hue;
        }
        this->show();
        vTaskDelay(80 / portTICK_PERIOD_MS);
    }
}