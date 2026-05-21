#pragma once

#include "CmdType.h"
#include "Utils/Color.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace LED {

template <size_t MaxEffects> class EffectEngine {
    static_assert(MaxEffects > 0 && (MaxEffects * 2U) <= UINT8_MAX,
                  "EffectEngine MaxEffects must fit uint8_t counters");

public:
    bool setEffect(const Cmd_s &_cmd, uint32_t _nowMs)
    {
        if (_cmd.ctrlNum == 0) {
            return false;
        }

        const uint8_t existing = findSameRange(_cmd.index, _cmd.ctrlNum);
        if (existing != INVALID_EFFECT) {
            removeAt(existing);
        } else if (activeCount_ == MaxEffects) {
            removeAt(0);
        }

        Effect_s &effect = effects_[activeCount_++];
        effect.type = _cmd.type;
        effect.index = _cmd.index;
        effect.len = _cmd.ctrlNum;
        effect.maxBrightness = _cmd.maxBrightness;
        effect.startMs = _nowMs;
        dirty_ = true;
        return true;
    }

    bool render(uint32_t _nowMs, RGB_s *_leds, size_t _total)
    {
        if (_leds == nullptr || _total == 0) {
            return false;
        }

        const bool shouldRender = dirty_ || hasDynamicEffect();
        if (!shouldRender) {
            return false;
        }

        clearDirtyRanges(_leds, _total);

        if (activeCount_ == 0) {
            dirty_ = false;
            pendingClearCount_ = 0;
            return true;
        }

        for (uint8_t i = 0; i < activeCount_; ++i) {
            renderEffect(effects_[i], _nowMs, _leds, _total);
        }

        dirty_ = false;
        pendingClearCount_ = 0;
        return true;
    }

    bool needsRefresh() const { return dirty_ || hasDynamicEffect(); }

    size_t activeCount() const { return activeCount_; }

    void clear()
    {
        for (uint8_t i = 0; i < activeCount_; ++i) {
            addPendingClear(effects_[i].index, effects_[i].len);
        }
        activeCount_ = 0;
        dirty_ = true;
    }

private:
    struct Effect_s {
        CmdType_e type = CmdType_e::OFF;
        uint8_t index = 0;
        uint8_t len = 0;
        uint8_t maxBrightness = 255;
        uint32_t startMs = 0;
    };

    std::array<Effect_s, MaxEffects> effects_{};
    uint8_t activeCount_ = 0;
    uint8_t pendingClearCount_ = 0;
    bool dirty_ = false;

    struct Range_s {
        uint16_t start = 0;
        uint16_t len = 0;
    };

    std::array<Range_s, MaxEffects> pendingClearRanges_{};

    static constexpr uint32_t FLOW_PERIOD_MS = 40U;
    static constexpr uint32_t SNAKE_PERIOD_MS = 20U;
    static constexpr uint32_t BLINK_PERIOD_MS = 500U;
    static constexpr uint8_t SNAKE_LEN = 8U;
    static constexpr uint8_t INVALID_EFFECT = UINT8_MAX;

    uint8_t findSameRange(uint8_t _index, uint8_t _len) const
    {
        for (uint8_t i = 0; i < activeCount_; ++i) {
            if (effects_[i].index == _index && effects_[i].len == _len) {
                return i;
            }
        }
        return INVALID_EFFECT;
    }

    void addPendingClear(uint8_t _index, uint8_t _len)
    {
        if (_len == 0) {
            return;
        }

        if (pendingClearCount_ < MaxEffects) {
            pendingClearRanges_[pendingClearCount_++] = Range_s{ _index, _len };
        } else {
            pendingClearRanges_[0] = Range_s{ 0, UINT16_MAX };
            pendingClearCount_ = 1;
        }
    }

    void removeAt(uint8_t _index)
    {
        if (_index >= activeCount_) {
            return;
        }
        addPendingClear(effects_[_index].index, effects_[_index].len);
        for (uint8_t i = _index; i + 1 < activeCount_; ++i) {
            effects_[i] = effects_[i + 1];
        }
        --activeCount_;
    }

    bool hasDynamicEffect() const
    {
        for (uint8_t i = 0; i < activeCount_; ++i) {
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

    static uint8_t scale8(uint8_t _value, uint8_t _brightness)
    {
        if (_brightness >= 255U) {
            return _value;
        }
        if (_brightness == 0U || _value == 0U) {
            return 0;
        }
        return static_cast<uint8_t>((static_cast<uint16_t>(_value) * _brightness + 255U) >> 8);
    }

    static RGB_s scaleColor8(const RGB_s &_color, uint8_t _brightness)
    {
        if (_brightness >= 255U) {
            return _color;
        }
        if (_brightness == 0U) {
            return RGB_s(0, 0, 0);
        }
        return RGB_s(scale8(_color.rgb.r, _brightness), scale8(_color.rgb.g, _brightness),
                     scale8(_color.rgb.b, _brightness));
    }

    static void fill(RGB_s *_leds, size_t _len, const RGB_s &_color)
    {
        for (size_t i = 0; i < _len; ++i) {
            _leds[i] = _color;
        }
    }

    static void fillScaled(RGB_s *_leds, size_t _len, const RGB_s &_color, uint8_t _brightness)
    {
        fill(_leds, _len, scaleColor8(_color, _brightness));
    }

    static void writeScaled(RGB_s *_leds, size_t _index, const RGB_s &_color, uint8_t _brightness)
    {
        _leds[_index] = scaleColor8(_color, _brightness);
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

    static void clearRange(RGB_s *_leds, size_t _total, uint16_t _index, uint16_t _len)
    {
        const size_t start = _index;
        if (start >= _total || _len == 0) {
            return;
        }

        const size_t requestedEnd = start + _len;
        const size_t end = requestedEnd < _total ? requestedEnd : _total;
        fill(&_leds[start], end - start, RGB_s(0, 0, 0));
    }

    void clearDirtyRanges(RGB_s *_leds, size_t _total)
    {
        std::array<Range_s, MaxEffects * 2> ranges{};
        uint8_t rangeCount = 0;

        const uint8_t pendingCount =
                static_cast<uint8_t>(std::min<size_t>(pendingClearCount_, pendingClearRanges_.size()));
        for (uint8_t i = 0; i < pendingCount && rangeCount < ranges.size(); ++i) {
            ranges[rangeCount++] = pendingClearRanges_[i];
        }

        for (uint8_t i = 0; i < activeCount_ && rangeCount < ranges.size(); ++i) {
            ranges[rangeCount++] = Range_s{ effects_[i].index, effects_[i].len };
        }

        sortRanges(ranges, rangeCount);

        for (uint8_t i = 0; i < rangeCount;) {
            uint16_t start = ranges[i].start;
            uint16_t end = static_cast<uint16_t>(ranges[i].start) + ranges[i].len;
            ++i;

            while (i < rangeCount && ranges[i].start <= end) {
                const uint16_t nextEnd = static_cast<uint16_t>(ranges[i].start) + ranges[i].len;
                end = std::max(nextEnd, end);
                ++i;
            }

            clearRange(_leds, _total, start, end - start);
        }
    }

    static void sortRanges(std::array<Range_s, MaxEffects * 2> &_ranges, uint8_t _count)
    {
        for (uint8_t i = 1; i < _count; ++i) {
            Range_s key = _ranges[i];
            uint8_t j = i;
            while (j > 0 && _ranges[j - 1].start > key.start) {
                _ranges[j] = _ranges[j - 1];
                --j;
            }
            _ranges[j] = key;
        }
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
            fillScaled(&_leds[start], end - start, RGB_s(0, 255, 0), _effect.maxBrightness);
            break;
        case CmdType_e::ON_IN_WARN:
            fillScaled(&_leds[start], end - start, RGB_s(255, 255, 0), _effect.maxBrightness);
            break;
        case CmdType_e::ON_IN_ERROR:
            fillScaled(&_leds[start], end - start, RGB_s(255, 0, 0), _effect.maxBrightness);
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
        fillScaled(&_leds[_start], _end - _start, COLORS[colorIndex], _effect.maxBrightness);
    }

    static void renderBlinkColor(const Effect_s &_effect, uint32_t _nowMs, RGB_s *_leds, size_t _start, size_t _end,
                                 const RGB_s &_color)
    {
        const bool on = (((_nowMs - _effect.startMs) / BLINK_PERIOD_MS) & 0x01U) == 0U;
        fillScaled(&_leds[_start], _end - _start, on ? _color : RGB_s(0, 0, 0), _effect.maxBrightness);
    }

    static void renderRainbowFlow(const Effect_s &_effect, uint32_t _nowMs, RGB_s *_leds, size_t _start, size_t _end,
                                  bool _reverse)
    {
        const uint8_t base = frame(_effect, _nowMs, FLOW_PERIOD_MS);
        for (size_t i = _start; i < _end; ++i) {
            const uint8_t offset = static_cast<uint8_t>(i - _start);
            writeScaled(_leds, i,
                        rainbowLut(_reverse ? static_cast<uint8_t>(base - offset) :
                                              static_cast<uint8_t>(base + offset)),
                        _effect.maxBrightness);
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
            _leds[_start + head] = scaleColor8(rainbowLut(hue), scale8(brightness, _effect.maxBrightness));
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
            _leds[i] = scaleColor8(rainbowLut(hue), scale8(brightness, _effect.maxBrightness));
        }
    }
};

} // namespace LED
