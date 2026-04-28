#pragma once

#include "CmdType.h"
#include "Utils/Color.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace LED {

template <size_t MaxEffects> class EffectEngine {
public:
    bool setEffect(const Cmd_s &_cmd, uint32_t _nowMs)
    {
        if (_cmd.ctrlNum == 0) {
            return false;
        }

        const size_t existing = findSameRange(_cmd.index, _cmd.ctrlNum);
        if (existing < MaxEffects) {
            removeAt(existing);
        } else if (activeCount_ == MaxEffects) {
            removeAt(0);
        }

        Effect_s &effect = effects_[activeCount_++];
        effect.type = _cmd.type;
        effect.index = _cmd.index;
        effect.len = _cmd.ctrlNum;
        effect.startMs = _nowMs;
        effect.dirty = true;
        dirty_ = true;
        return true;
    }

    bool render(uint32_t _nowMs, RGB_s *_leds, size_t _total)
    {
        if (_leds == nullptr || _total == 0 || activeCount_ == 0) {
            return false;
        }

        const bool shouldRender = dirty_ || hasDynamicEffect();
        if (!shouldRender) {
            return false;
        }

        fill(_leds, _total, RGB_s(0, 0, 0));

        for (size_t i = 0; i < activeCount_; ++i) {
            renderEffect(effects_[i], _nowMs, _leds, _total);
            effects_[i].dirty = false;
        }

        dirty_ = false;
        return true;
    }

    bool needsRefresh() const { return dirty_ || hasDynamicEffect(); }

    size_t activeCount() const { return activeCount_; }

    void clear()
    {
        activeCount_ = 0;
        dirty_ = true;
    }

private:
    struct Effect_s {
        CmdType_e type = CmdType_e::OFF;
        uint8_t index = 0;
        uint8_t len = 0;
        uint32_t startMs = 0;
        bool dirty = false;
    };

    std::array<Effect_s, MaxEffects> effects_{};
    size_t activeCount_ = 0;
    bool dirty_ = false;

    static constexpr uint32_t FLOW_PERIOD_MS = 40U;
    static constexpr uint32_t SNAKE_PERIOD_MS = 20U;
    static constexpr uint32_t BLINK_PERIOD_MS = 500U;
    static constexpr uint8_t SNAKE_LEN = 8U;

    size_t findSameRange(uint8_t _index, uint8_t _len) const
    {
        for (size_t i = 0; i < activeCount_; ++i) {
            if (effects_[i].index == _index && effects_[i].len == _len) {
                return i;
            }
        }
        return MaxEffects;
    }

    void removeAt(size_t _index)
    {
        if (_index >= activeCount_) {
            return;
        }
        for (size_t i = _index; i + 1 < activeCount_; ++i) {
            effects_[i] = effects_[i + 1];
        }
        --activeCount_;
    }

    bool hasDynamicEffect() const
    {
        for (size_t i = 0; i < activeCount_; ++i) {
            if (isDynamic(effects_[i].type)) {
                return true;
            }
        }
        return false;
    }

    static bool isDynamic(CmdType_e _type)
    {
        switch (_type) {
        case CmdType_e::BLINK_RGB:
        case CmdType_e::BLINK_RED:
        case CmdType_e::BLINK_GREEN:
        case CmdType_e::BLINK_BLUE:
        case CmdType_e::RAINBOW_FLOW:
        case CmdType_e::RAINBOW_FLOW_REVERSE:
        case CmdType_e::RAINBOW_FLOW_SNAKE:
        case CmdType_e::RAINBOW_BREATH:
            return true;
        default:
            return false;
        }
    }

    static RGB_s rainbowWheel(uint8_t _position)
    {
        _position = 255U - _position;
        if (_position < 85U) {
            return RGB_s(static_cast<uint8_t>(255U - (_position * 3U)), 0, static_cast<uint8_t>(_position * 3U));
        }
        if (_position < 170U) {
            _position -= 85U;
            return RGB_s(0, static_cast<uint8_t>(_position * 3U), static_cast<uint8_t>(255U - (_position * 3U)));
        }

        _position -= 170U;
        return RGB_s(static_cast<uint8_t>(_position * 3U), static_cast<uint8_t>(255U - (_position * 3U)), 0);
    }

    static const RGB_s &rainbowLut(uint8_t _position)
    {
        static const std::array<RGB_s, 256> LUT = [] {
            std::array<RGB_s, 256> table{};
            for (uint16_t i = 0; i < 256U; ++i) {
                table[static_cast<size_t>(i)] = rainbowWheel(static_cast<uint8_t>(i));
            }
            return table;
        }();

        return LUT[_position];
    }

    static RGB_s scaleColor8(const RGB_s &_color, uint8_t _brightness)
    {
        return RGB_s(static_cast<uint8_t>((static_cast<uint16_t>(_color.rgb.r) * _brightness) >> 8),
                     static_cast<uint8_t>((static_cast<uint16_t>(_color.rgb.g) * _brightness) >> 8),
                     static_cast<uint8_t>((static_cast<uint16_t>(_color.rgb.b) * _brightness) >> 8));
    }

    static void fill(RGB_s *_leds, size_t _len, const RGB_s &_color)
    {
        for (size_t i = 0; i < _len; ++i) {
            _leds[i] = _color;
        }
    }

    static size_t clampedEnd(const Effect_s &_effect, size_t _total)
    {
        const size_t start = _effect.index;
        if (start >= _total) {
            return start;
        }

        const size_t requestedEnd = start + _effect.len;
        return requestedEnd < _total ? requestedEnd : _total;
    }

    static uint8_t frame(const Effect_s &_effect, uint32_t _nowMs, uint32_t _periodMs)
    {
        return static_cast<uint8_t>((_nowMs - _effect.startMs) / _periodMs);
    }

    static void renderEffect(const Effect_s &_effect, uint32_t _nowMs, RGB_s *_leds, size_t _total)
    {
        const size_t start = _effect.index;
        const size_t end = clampedEnd(_effect, _total);
        if (start >= end) {
            return;
        }

        switch (_effect.type) {
        case CmdType_e::OFF:
            fill(&_leds[start], end - start, RGB_s(0, 0, 0));
            break;
        case CmdType_e::ON_IN_NORMAL:
            fill(&_leds[start], end - start, RGB_s(0, 255, 0));
            break;
        case CmdType_e::ON_IN_WARN:
            fill(&_leds[start], end - start, RGB_s(255, 255, 0));
            break;
        case CmdType_e::ON_IN_ERROR:
            fill(&_leds[start], end - start, RGB_s(255, 0, 0));
            break;
        case CmdType_e::BLINK_RGB:
            renderBlinkRGB(_effect, _nowMs, _leds, start, end);
            break;
        case CmdType_e::BLINK_RED:
            renderBlinkColor(_effect, _nowMs, _leds, start, end, RGB_s(255, 0, 0));
            break;
        case CmdType_e::BLINK_GREEN:
            renderBlinkColor(_effect, _nowMs, _leds, start, end, RGB_s(0, 255, 0));
            break;
        case CmdType_e::BLINK_BLUE:
            renderBlinkColor(_effect, _nowMs, _leds, start, end, RGB_s(0, 0, 255));
            break;
        case CmdType_e::RAINBOW_FLOW:
            renderRainbowFlow(_effect, _nowMs, _leds, start, end, false);
            break;
        case CmdType_e::RAINBOW_FLOW_REVERSE:
            renderRainbowFlow(_effect, _nowMs, _leds, start, end, true);
            break;
        case CmdType_e::RAINBOW_FLOW_SNAKE:
            renderRainbowSnake(_effect, _nowMs, _leds, start, end);
            break;
        case CmdType_e::RAINBOW_BREATH:
            renderRainbowBreath(_effect, _nowMs, _leds, start, end);
            break;
        default:
            break;
        }
    }

    static void renderBlinkRGB(const Effect_s &_effect, uint32_t _nowMs, RGB_s *_leds, size_t _start, size_t _end)
    {
        static const std::array<RGB_s, 3> COLORS = { RGB_s(255, 0, 0), RGB_s(0, 255, 0), RGB_s(0, 0, 255) };
        const uint8_t colorIndex = static_cast<uint8_t>((_nowMs - _effect.startMs) / BLINK_PERIOD_MS) % COLORS.size();
        fill(&_leds[_start], _end - _start, COLORS[colorIndex]);
    }

    static void renderBlinkColor(const Effect_s &_effect, uint32_t _nowMs, RGB_s *_leds, size_t _start, size_t _end,
                                 const RGB_s &_color)
    {
        const bool on = (((_nowMs - _effect.startMs) / BLINK_PERIOD_MS) & 0x01U) == 0U;
        fill(&_leds[_start], _end - _start, on ? _color : RGB_s(0, 0, 0));
    }

    static void renderRainbowFlow(const Effect_s &_effect, uint32_t _nowMs, RGB_s *_leds, size_t _start, size_t _end,
                                  bool _reverse)
    {
        const uint8_t base = frame(_effect, _nowMs, FLOW_PERIOD_MS);
        for (size_t i = _start; i < _end; ++i) {
            const uint8_t offset = static_cast<uint8_t>(i - _start);
            _leds[i] = rainbowLut(_reverse ? static_cast<uint8_t>(base - offset) : static_cast<uint8_t>(base + offset));
        }
    }

    static void renderRainbowSnake(const Effect_s &_effect, uint32_t _nowMs, RGB_s *_leds, size_t _start, size_t _end)
    {
        const uint8_t len = static_cast<uint8_t>(_end - _start);
        const uint8_t bodyLen = (len < SNAKE_LEN) ? len : SNAKE_LEN;
        const uint8_t step = frame(_effect, _nowMs, SNAKE_PERIOD_MS);
        uint8_t head = static_cast<uint8_t>(step % len);

        for (uint8_t body = 0; body < bodyLen; ++body) {
            const uint8_t brightness = static_cast<uint8_t>(255U - ((static_cast<uint16_t>(body) * 255U) / bodyLen));
            const uint8_t hue = static_cast<uint8_t>((step * 4U) + (body * 18U));
            _leds[_start + head] = scaleColor8(rainbowLut(hue), brightness);
            head = (head == 0U) ? static_cast<uint8_t>(len - 1U) : static_cast<uint8_t>(head - 1U);
        }
    }

    static void renderRainbowBreath(const Effect_s &_effect, uint32_t _nowMs, RGB_s *_leds, size_t _start, size_t _end)
    {
        const uint8_t step = frame(_effect, _nowMs, FLOW_PERIOD_MS);
        const uint8_t brightness = (step < 128U) ? static_cast<uint8_t>(step * 2U) :
                                                   static_cast<uint8_t>((255U - step) * 2U);

        for (size_t i = _start; i < _end; ++i) {
            const uint8_t hue = static_cast<uint8_t>(step + static_cast<uint8_t>(i - _start));
            _leds[i] = scaleColor8(rainbowLut(hue), brightness);
        }
    }
};

} // namespace LED
