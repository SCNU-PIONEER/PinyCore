#pragma once

#include <algorithm>

#include "math8.hpp"
#include "scale8.hpp"

namespace LED {

struct RGB_s {
    union RGB_u {
        struct {
            union {
                uint8_t r;   ///< Red channel value
                uint8_t red; ///< @copydoc rgb.r
            };
            union {
                uint8_t g;     ///< Green channel value
                uint8_t green; ///< @copydoc rgb.g
            };
            union {
                uint8_t b;    ///< Blue channel value
                uint8_t blue; ///< @copydoc rgb.b
            };
        };
        /// Access the red, green, and blue data as an array.
        /// Where:
        /// * `raw[0]` is the red value
        /// * `raw[1]` is the green value
        /// * `raw[2]` is the blue value
        uint8_t raw[3];
    } rgb;

    /// Predefined RGB colors (HTMLColorCode)
    // NOLINTBEGIN
    enum class Color_e : uint32_t {
        AliceBlue = 0xF0F8FF,            ///< @htmlcolorblock{F0F8FF}
        Amethyst = 0x9966CC,             ///< @htmlcolorblock{9966CC}
        AntiqueWhite = 0xFAEBD7,         ///< @htmlcolorblock{FAEBD7}
        Aqua = 0x00FFFF,                 ///< @htmlcolorblock{00FFFF}
        Aquamarine = 0x7FFFD4,           ///< @htmlcolorblock{7FFFD4}
        Azure = 0xF0FFFF,                ///< @htmlcolorblock{F0FFFF}
        Beige = 0xF5F5DC,                ///< @htmlcolorblock{F5F5DC}
        Bisque = 0xFFE4C4,               ///< @htmlcolorblock{FFE4C4}
        Black = 0x000000,                ///< @htmlcolorblock{000000}
        BlanchedAlmond = 0xFFEBCD,       ///< @htmlcolorblock{FFEBCD}
        Blue = 0x0000FF,                 ///< @htmlcolorblock{0000FF}
        BlueViolet = 0x8A2BE2,           ///< @htmlcolorblock{8A2BE2}
        Brown = 0xA52A2A,                ///< @htmlcolorblock{A52A2A}
        BurlyWood = 0xDEB887,            ///< @htmlcolorblock{DEB887}
        CadetBlue = 0x5F9EA0,            ///< @htmlcolorblock{5F9EA0}
        Chartreuse = 0x7FFF00,           ///< @htmlcolorblock{7FFF00}
        Chocolate = 0xD2691E,            ///< @htmlcolorblock{D2691E}
        Coral = 0xFF7F50,                ///< @htmlcolorblock{FF7F50}
        CornflowerBlue = 0x6495ED,       ///< @htmlcolorblock{6495ED}
        Cornsilk = 0xFFF8DC,             ///< @htmlcolorblock{FFF8DC}
        Crimson = 0xDC143C,              ///< @htmlcolorblock{DC143C}
        Cyan = 0x00FFFF,                 ///< @htmlcolorblock{00FFFF}
        DarkBlue = 0x00008B,             ///< @htmlcolorblock{00008B}
        DarkCyan = 0x008B8B,             ///< @htmlcolorblock{008B8B}
        DarkGoldenrod = 0xB8860B,        ///< @htmlcolorblock{B8860B}
        DarkGray = 0xA9A9A9,             ///< @htmlcolorblock{A9A9A9}
        DarkGrey = 0xA9A9A9,             ///< @htmlcolorblock{A9A9A9}
        DarkGreen = 0x006400,            ///< @htmlcolorblock{006400}
        DarkKhaki = 0xBDB76B,            ///< @htmlcolorblock{BDB76B}
        DarkMagenta = 0x8B008B,          ///< @htmlcolorblock{8B008B}
        DarkOliveGreen = 0x556B2F,       ///< @htmlcolorblock{556B2F}
        DarkOrange = 0xFF8C00,           ///< @htmlcolorblock{FF8C00}
        DarkOrchid = 0x9932CC,           ///< @htmlcolorblock{9932CC}
        DarkRed = 0x8B0000,              ///< @htmlcolorblock{8B0000}
        DarkSalmon = 0xE9967A,           ///< @htmlcolorblock{E9967A}
        DarkSeaGreen = 0x8FBC8F,         ///< @htmlcolorblock{8FBC8F}
        DarkSlateBlue = 0x483D8B,        ///< @htmlcolorblock{483D8B}
        DarkSlateGray = 0x2F4F4F,        ///< @htmlcolorblock{2F4F4F}
        DarkSlateGrey = 0x2F4F4F,        ///< @htmlcolorblock{2F4F4F}
        DarkTurquoise = 0x00CED1,        ///< @htmlcolorblock{00CED1}
        DarkViolet = 0x9400D3,           ///< @htmlcolorblock{9400D3}
        DeepPink = 0xFF1493,             ///< @htmlcolorblock{FF1493}
        DeepSkyBlue = 0x00BFFF,          ///< @htmlcolorblock{00BFFF}
        DimGray = 0x696969,              ///< @htmlcolorblock{696969}
        DimGrey = 0x696969,              ///< @htmlcolorblock{696969}
        DodgerBlue = 0x1E90FF,           ///< @htmlcolorblock{1E90FF}
        FireBrick = 0xB22222,            ///< @htmlcolorblock{B22222}
        FloralWhite = 0xFFFAF0,          ///< @htmlcolorblock{FFFAF0}
        ForestGreen = 0x228B22,          ///< @htmlcolorblock{228B22}
        Fuchsia = 0xFF00FF,              ///< @htmlcolorblock{FF00FF}
        Gainsboro = 0xDCDCDC,            ///< @htmlcolorblock{DCDCDC}
        GhostWhite = 0xF8F8FF,           ///< @htmlcolorblock{F8F8FF}
        Gold = 0xFFD700,                 ///< @htmlcolorblock{FFD700}
        Goldenrod = 0xDAA520,            ///< @htmlcolorblock{DAA520}
        Gray = 0x808080,                 ///< @htmlcolorblock{808080}
        Grey = 0x808080,                 ///< @htmlcolorblock{808080}
        Green = 0x008000,                ///< @htmlcolorblock{008000}
        GreenYellow = 0xADFF2F,          ///< @htmlcolorblock{ADFF2F}
        Honeydew = 0xF0FFF0,             ///< @htmlcolorblock{F0FFF0}
        HotPink = 0xFF69B4,              ///< @htmlcolorblock{FF69B4}
        IndianRed = 0xCD5C5C,            ///< @htmlcolorblock{CD5C5C}
        Indigo = 0x4B0082,               ///< @htmlcolorblock{4B0082}
        Ivory = 0xFFFFF0,                ///< @htmlcolorblock{FFFFF0}
        Khaki = 0xF0E68C,                ///< @htmlcolorblock{F0E68C}
        Lavender = 0xE6E6FA,             ///< @htmlcolorblock{E6E6FA}
        LavenderBlush = 0xFFF0F5,        ///< @htmlcolorblock{FFF0F5}
        LawnGreen = 0x7CFC00,            ///< @htmlcolorblock{7CFC00}
        LemonChiffon = 0xFFFACD,         ///< @htmlcolorblock{FFFACD}
        LightBlue = 0xADD8E6,            ///< @htmlcolorblock{ADD8E6}
        LightCoral = 0xF08080,           ///< @htmlcolorblock{F08080}
        LightCyan = 0xE0FFFF,            ///< @htmlcolorblock{E0FFFF}
        LightGoldenrodYellow = 0xFAFAD2, ///< @htmlcolorblock{FAFAD2}
        LightGreen = 0x90EE90,           ///< @htmlcolorblock{90EE90}
        LightGrey = 0xD3D3D3,            ///< @htmlcolorblock{D3D3D3}
        LightPink = 0xFFB6C1,            ///< @htmlcolorblock{FFB6C1}
        LightSalmon = 0xFFA07A,          ///< @htmlcolorblock{FFA07A}
        LightSeaGreen = 0x20B2AA,        ///< @htmlcolorblock{20B2AA}
        LightSkyBlue = 0x87CEFA,         ///< @htmlcolorblock{87CEFA}
        LightSlateGray = 0x778899,       ///< @htmlcolorblock{778899}
        LightSlateGrey = 0x778899,       ///< @htmlcolorblock{778899}
        LightSteelBlue = 0xB0C4DE,       ///< @htmlcolorblock{B0C4DE}
        LightYellow = 0xFFFFE0,          ///< @htmlcolorblock{FFFFE0}
        Lime = 0x00FF00,                 ///< @htmlcolorblock{00FF00}
        LimeGreen = 0x32CD32,            ///< @htmlcolorblock{32CD32}
        Linen = 0xFAF0E6,                ///< @htmlcolorblock{FAF0E6}
        Magenta = 0xFF00FF,              ///< @htmlcolorblock{FF00FF}
        Maroon = 0x800000,               ///< @htmlcolorblock{800000}
        MediumAquamarine = 0x66CDAA,     ///< @htmlcolorblock{66CDAA}
        MediumBlue = 0x0000CD,           ///< @htmlcolorblock{0000CD}
        MediumOrchid = 0xBA55D3,         ///< @htmlcolorblock{BA55D3}
        MediumPurple = 0x9370DB,         ///< @htmlcolorblock{9370DB}
        MediumSeaGreen = 0x3CB371,       ///< @htmlcolorblock{3CB371}
        MediumSlateBlue = 0x7B68EE,      ///< @htmlcolorblock{7B68EE}
        MediumSpringGreen = 0x00FA9A,    ///< @htmlcolorblock{00FA9A}
        MediumTurquoise = 0x48D1CC,      ///< @htmlcolorblock{48D1CC}
        MediumVioletRed = 0xC71585,      ///< @htmlcolorblock{C71585}
        MidnightBlue = 0x191970,         ///< @htmlcolorblock{191970}
        MintCream = 0xF5FFFA,            ///< @htmlcolorblock{F5FFFA}
        MistyRose = 0xFFE4E1,            ///< @htmlcolorblock{FFE4E1}
        Moccasin = 0xFFE4B5,             ///< @htmlcolorblock{FFE4B5}
        NavajoWhite = 0xFFDEAD,          ///< @htmlcolorblock{FFDEAD}
        Navy = 0x000080,                 ///< @htmlcolorblock{000080}
        OldLace = 0xFDF5E6,              ///< @htmlcolorblock{FDF5E6}
        Olive = 0x808000,                ///< @htmlcolorblock{808000}
        OliveDrab = 0x6B8E23,            ///< @htmlcolorblock{6B8E23}
        Orange = 0xFFA500,               ///< @htmlcolorblock{FFA500}
        OrangeRed = 0xFF4500,            ///< @htmlcolorblock{FF4500}
        Orchid = 0xDA70D6,               ///< @htmlcolorblock{DA70D6}
        PaleGoldenrod = 0xEEE8AA,        ///< @htmlcolorblock{EEE8AA}
        PaleGreen = 0x98FB98,            ///< @htmlcolorblock{98FB98}
        PaleTurquoise = 0xAFEEEE,        ///< @htmlcolorblock{AFEEEE}
        PaleVioletRed = 0xDB7093,        ///< @htmlcolorblock{DB7093}
        PapayaWhip = 0xFFEFD5,           ///< @htmlcolorblock{FFEFD5}
        PeachPuff = 0xFFDAB9,            ///< @htmlcolorblock{FFDAB9}
        Peru = 0xCD853F,                 ///< @htmlcolorblock{CD853F}
        Pink = 0xFFC0CB,                 ///< @htmlcolorblock{FFC0CB}
        Plaid = 0xCC5533,                ///< @htmlcolorblock{CC5533}
        Plum = 0xDDA0DD,                 ///< @htmlcolorblock{DDA0DD}
        PowderBlue = 0xB0E0E6,           ///< @htmlcolorblock{B0E0E6}
        Purple = 0x800080,               ///< @htmlcolorblock{800080}
        Red = 0xFF0000,                  ///< @htmlcolorblock{FF0000}
        RosyBrown = 0xBC8F8F,            ///< @htmlcolorblock{BC8F8F}
        RoyalBlue = 0x4169E1,            ///< @htmlcolorblock{4169E1}
        SaddleBrown = 0x8B4513,          ///< @htmlcolorblock{8B4513}
        Salmon = 0xFA8072,               ///< @htmlcolorblock{FA8072}
        SandyBrown = 0xF4A460,           ///< @htmlcolorblock{F4A460}
        SeaGreen = 0x2E8B57,             ///< @htmlcolorblock{2E8B57}
        Seashell = 0xFFF5EE,             ///< @htmlcolorblock{FFF5EE}
        Sienna = 0xA0522D,               ///< @htmlcolorblock{A0522D}
        Silver = 0xC0C0C0,               ///< @htmlcolorblock{C0C0C0}
        SkyBlue = 0x87CEEB,              ///< @htmlcolorblock{87CEEB}
        SlateBlue = 0x6A5ACD,            ///< @htmlcolorblock{6A5ACD}
        SlateGray = 0x708090,            ///< @htmlcolorblock{708090}
        SlateGrey = 0x708090,            ///< @htmlcolorblock{708090}
        Snow = 0xFFFAFA,                 ///< @htmlcolorblock{FFFAFA}
        SpringGreen = 0x00FF7F,          ///< @htmlcolorblock{00FF7F}
        SteelBlue = 0x4682B4,            ///< @htmlcolorblock{4682B4}
        Tan = 0xD2B48C,                  ///< @htmlcolorblock{D2B48C}
        Teal = 0x008080,                 ///< @htmlcolorblock{008080}
        Thistle = 0xD8BFD8,              ///< @htmlcolorblock{D8BFD8}
        Tomato = 0xFF6347,               ///< @htmlcolorblock{FF6347}
        Turquoise = 0x40E0D0,            ///< @htmlcolorblock{40E0D0}
        Violet = 0xEE82EE,               ///< @htmlcolorblock{EE82EE}
        Wheat = 0xF5DEB3,                ///< @htmlcolorblock{F5DEB3}
        White = 0xFFFFFF,                ///< @htmlcolorblock{FFFFFF}
        WhiteSmoke = 0xF5F5F5,           ///< @htmlcolorblock{F5F5F5}
        Yellow = 0xFFFF00,               ///< @htmlcolorblock{FFFF00}
        YellowGreen = 0x9ACD32,          ///< @htmlcolorblock{9ACD32}

        // LED RGB color that roughly approximates
        // the color of incandescent fairy lights,
        // assuming that you're using FastLED
        // color correction on your LEDs (recommended).
        FairyLight = 0xFFE42D, ///< @htmlcolorblock{FFE42D}

        // If you are using no color correction, use this
        FairyLightNCC = 0xFF9D2A ///< @htmlcolorblock{FFE42D}

    };
    // NOLINTEND

    /// Array access operator to index into the RGB object
    /// @param x the index to retrieve (0-2)
    /// @returns the RGB::raw value for the given index
    uint8_t &operator[](uint8_t _x) { return rgb.raw[_x]; }

    /// @copydoc operator[]
    const uint8_t &operator[](uint8_t _x) const { return rgb.raw[_x]; }

    /// construct an RGB object with all values set to zero
    /// @param ir input red
    /// @param ig input green
    /// @param ib input blue
    RGB_s(uint8_t _ir = 0, uint8_t _ig = 0, uint8_t _ib = 0)
    {
        rgb.r = _ir;
        rgb.g = _ig;
        rgb.b = _ib;
    }

    /// Allow construction from 32-bit (really 24-bit) bit 0xRRGGBB color code
    /// @param colorCode a packed 24 bit color code
    RGB_s(uint32_t _colorCode)
    {
        rgb.r = (_colorCode >> 16) & 0xFF;
        rgb.g = (_colorCode >> 8) & 0xFF;
        rgb.b = (_colorCode >> 0) & 0xFF;
    }
    RGB_s &operator=(const uint32_t _colorCode)
    {
        rgb.r = (_colorCode >> 16) & 0xFF;
        rgb.g = (_colorCode >> 8) & 0xFF;
        rgb.b = (_colorCode >> 0) & 0xFF;
        return *this;
    }

    /// Allow copy construction
    RGB_s(const RGB_s &_rhs) = default;
    RGB_s &operator=(const RGB_s &_rhs) = default;

    /// Allow assignment from red, green, and blue
    /// @param nr new red value
    /// @param ng new green value
    /// @param nb new blue value
    RGB_s &setRGB(uint8_t _nr, uint8_t _ng, uint8_t _nb)

    {
        rgb.r = _nr;
        rgb.g = _ng;
        rgb.b = _nb;
        return *this;
    }

    /// Allow assignment from 32-bit (really 24-bit) 0xRRGGBB color code
    /// @param colorCode a packed 24 bit color code
    RGB_s &setColorCode(uint32_t _colorCode)
    {
        rgb.r = (_colorCode >> 16) & 0xFF;
        rgb.g = (_colorCode >> 8) & 0xFF;
        rgb.b = (_colorCode >> 0) & 0xFF;
        return *this;
    }

    /// Allow assignment from predefined color code
    /// @param colorCode a predefined color code
    RGB_s &setColorCode(Color_e _colorCode)
    {
        uint32_t c = static_cast<uint32_t>(_colorCode);
        rgb.r = (c >> 16) & 0xFF;
        rgb.g = (c >> 8) & 0xFF;
        rgb.b = (c >> 0) & 0xFF;
        return *this;
    }

    /// Add one RGB_s to another, saturating at 0xFF for each channel
    RGB_s &operator+=(const RGB_s &_rhs)
    {
        rgb.r = qadd8(rgb.r, _rhs.rgb.r);
        rgb.g = qadd8(rgb.g, _rhs.rgb.g);
        rgb.b = qadd8(rgb.b, _rhs.rgb.b);
        return *this;
    }

    /// Add a constant to each channel, saturating at 0xFF.
    /// @note This is NOT an operator+= overload because the compiler
    /// can't usefully decide when it's being passed a 32-bit
    /// constant (e.rgb.g. RGB_s::Red) and an 8-bit one (RGB_s::Blue)
    RGB_s &addToRGB(uint8_t _d)
    {
        rgb.r = qadd8(rgb.r, _d);
        rgb.g = qadd8(rgb.g, _d);
        rgb.b = qadd8(rgb.b, _d);
        return *this;
    }

    /// Subtract one RGB_s from another, saturating at 0x00 for each channel
    RGB_s &operator-=(const RGB_s &_rhs)
    {
        rgb.r = qsub8(rgb.r, _rhs.rgb.r);
        rgb.g = qsub8(rgb.g, _rhs.rgb.g);
        rgb.b = qsub8(rgb.b, _rhs.rgb.b);
        return *this;
    }

    /// Subtract a constant from each channel, saturating at 0x00.
    /// @note This is NOT an operator+= overload because the compiler
    /// can't usefully decide when it's being passed a 32-bit
    /// constant (e.rgb.g. RGB_s::Red) and an 8-bit one (RGB_s::Blue)
    RGB_s &subtractFromRGB(uint8_t _d)
    {
        rgb.r = qsub8(rgb.r, _d);
        rgb.g = qsub8(rgb.g, _d);
        rgb.b = qsub8(rgb.b, _d);
        return *this;
    }

    /// Subtract a constant of '1' from each channel, saturating at 0x00
    RGB_s &operator--() __attribute__((always_inline))
    {
        subtractFromRGB(1);
        return *this;
    }

    /// @copydoc operator--
    RGB_s operator--(int) __attribute__((always_inline))
    {
        RGB_s retval(*this);
        --(*this);
        return retval;
    }

    /// Add a constant of '1' from each channel, saturating at 0xFF
    RGB_s &operator++() __attribute__((always_inline))
    {
        addToRGB(1);
        return *this;
    }

    /// @copydoc operator++
    RGB_s operator++(int) __attribute__((always_inline))
    {
        RGB_s retval(*this);
        ++(*this);
        return retval;
    }

    /// Divide each of the channels by a constant
    RGB_s &operator/=(uint8_t _d)
    {
        rgb.r /= _d;
        rgb.g /= _d;
        rgb.b /= _d;
        return *this;
    }

    /// Right shift each of the channels by a constant
    RGB_s &operator>>=(uint8_t _d)
    {
        rgb.r >>= _d;
        rgb.g >>= _d;
        rgb.b >>= _d;
        return *this;
    }

    /// Multiply each of the channels by a constant,
    /// saturating each channel at 0xFF.
    RGB_s &operator*=(uint8_t _d)
    {
        rgb.r = qmul8(rgb.r, _d);
        rgb.g = qmul8(rgb.g, _d);
        rgb.b = qmul8(rgb.b, _d);
        return *this;
    }

    /// Scale down a RGB to N/256ths of it's current brightness using
    /// "video" dimming rules. "Video" dimming rules means that unless the scale factor
    /// is ZERO each channel is guaranteed NOT to dim down to zero.  If it's already
    /// nonzero, it'll stay nonzero, even if that means the hue shifts a little
    /// at low brightness levels.
    /// @see nscale8x3Video
    RGB_s &nscale8Video(uint8_t _scaledown)
    {
        nscale8x3Video(rgb.r, rgb.g, rgb.b, _scaledown);
        return *this;
    }

    /// %= is a synonym for nscale8Video().  Think of it is scaling down
    /// by "a percentage"
    RGB_s &operator%=(uint8_t _scaledown)
    {
        nscale8x3Video(rgb.r, rgb.g, rgb.b, _scaledown);
        return *this;
    }

    /// fadeLightBy is a synonym for nscale8Video(), as a fade instead of a scale
    /// @param _fadefactor the amount to fade, sent to nscale8Video() as (255 - _fadefactor)
    RGB_s &fadeLightBy(uint8_t _fadefactor)
    {
        nscale8x3Video(rgb.r, rgb.g, rgb.b, 255 - _fadefactor);
        return *this;
    }

    /// Scale down a RGB to N/256ths of its current brightness, using
    /// "plain math" dimming rules. "Plain math" dimming rules means that the low light
    /// levels may dim all the way to 100% black.
    /// @see nscale8x3
    RGB_s &nscale8(uint8_t _scaledown)
    {
        nscale8x3(rgb.r, rgb.g, rgb.b, _scaledown);
        return *this;
    }

    /// Scale down a RGB to N/256ths of its current brightness, using
    /// "plain math" dimming rules. "Plain math" dimming rules means that the low light
    /// levels may dim all the way to 100% black.
    /// @see ::scale8
    RGB_s &nscale8(const RGB_s &_scaledown)
    {
        rgb.r = LED::scale8(rgb.r, _scaledown.rgb.r);
        rgb.g = LED::scale8(rgb.g, _scaledown.rgb.g);
        rgb.b = LED::scale8(rgb.b, _scaledown.rgb.b);
        return *this;
    }

    /// Return a RGB_s object that is a scaled down version of this object
    RGB_s scale8(uint8_t _scaledown) const
    {
        RGB_s out = *this;
        nscale8x3(out.rgb.r, out.rgb.g, out.rgb.b, _scaledown);
        return out;
    }

    /// Return a RGB_s object that is a scaled down version of this object
    RGB_s scale8(const RGB_s &_scaledown) const
    {
        RGB_s out;
        out.rgb.r = LED::scale8(rgb.r, _scaledown.rgb.r);
        out.rgb.g = LED::scale8(rgb.g, _scaledown.rgb.g);
        out.rgb.b = LED::scale8(rgb.b, _scaledown.rgb.b);
        return out;
    }

    /// fadeToBlackBy is a synonym for nscale8(), as a fade instead of a scale
    /// @param _fadefactor the amount to fade, sent to nscale8() as (255 - _fadefactor)
    RGB_s &fadeToBlackBy(uint8_t _fadefactor)
    {
        nscale8x3(rgb.r, rgb.g, rgb.b, 255 - _fadefactor);
        return *this;
    }

    /// "or" operator brings each channel up to the higher of the two values
    RGB_s &operator|=(const RGB_s &_rhs)
    {
        rgb.r = std::max(_rhs.rgb.r, rgb.r);
        rgb.g = std::max(_rhs.rgb.g, rgb.g);
        rgb.b = std::max(_rhs.rgb.b, rgb.b);
        return *this;
    }

    /// @copydoc operator|=
    RGB_s &operator|=(uint8_t _d)
    {
        rgb.r = std::max(_d, rgb.r);
        rgb.g = std::max(_d, rgb.g);
        rgb.b = std::max(_d, rgb.b);
        return *this;
    }

    /// "and" operator brings each channel down to the lower of the two values
    RGB_s &operator&=(const RGB_s &_rhs)
    {
        rgb.r = std::min(_rhs.rgb.r, rgb.r);
        rgb.g = std::min(_rhs.rgb.g, rgb.g);
        rgb.b = std::min(_rhs.rgb.b, rgb.b);
        return *this;
    }

    /// @copydoc operator&=
    RGB_s &operator&=(uint8_t _d)
    {
        rgb.r = std::min(_d, rgb.r);
        rgb.g = std::min(_d, rgb.g);
        rgb.b = std::min(_d, rgb.b);
        return *this;
    }

    /// This allows testing a RGB_s for zero-ness
    explicit operator bool() const __attribute__((always_inline)) { return rgb.r || rgb.g || rgb.b; }

    /// Converts a RGB_s to a 32-bit color having an alpha of 255.
    explicit operator uint32_t() const
    {
        return uint32_t{ 0xff000000 } | (uint32_t{ rgb.r } << 16) | (uint32_t{ rgb.g } << 8) | uint32_t{ rgb.b };
    }

    /// Invert each channel
    RGB_s operator-() const
    {
        RGB_s retval;
        retval.rgb.r = 255 - rgb.r;
        retval.rgb.g = 255 - rgb.g;
        retval.rgb.b = 255 - rgb.b;
        return retval;
    }

    /// Get the "luma" of a RGB_s object. In other words, roughly how much
    /// light the RGB_s pixel is putting out (from 0 to 255).
    uint8_t getLuma() const
    {
        //Y' = 0.2126 R' + 0.7152 G' + 0.0722 B'
        //     54            183       18 (!)

        uint8_t luma = LED::scale8(rgb.r, 54) + LED::scale8(rgb.g, 183) + LED::scale8(rgb.b, 18);
        return luma;
    }

    /// Get the average of the R, G, and B values
    uint8_t getAverageLight() const
    {
        const uint8_t eightyfive = 86;
        uint8_t avg = LED::scale8(rgb.r, eightyfive) + LED::scale8(rgb.g, eightyfive) + LED::scale8(rgb.b, eightyfive);
        return avg;
    }

    /// Maximize the brightness of this RGB_s object.
    /// This makes the individual color channels as bright as possible
    /// while keeping the same value differences between channels.
    /// @note This does not keep the same ratios between channels,
    /// just the same difference in absolute values.
    void maximizeBrightness(uint8_t _limit = 255)
    {
        uint8_t max = rgb.red;
        max = std::max(rgb.green, max);
        max = std::max(rgb.blue, max);

        // stop div/0 when color is black
        if (max > 0) {
            uint16_t factor = ((uint16_t)(_limit) * 256) / max;
            rgb.red = (rgb.red * factor) / 256;
            rgb.green = (rgb.green * factor) / 256;
            rgb.blue = (rgb.blue * factor) / 256;
        }
    }

    /// Return a new RGB_s object after performing a linear interpolation between this object and the passed in object
    RGB_s lerp8(const RGB_s &_other, int _frac) const
    {
        RGB_s ret;

        ret.rgb.r = lerp8by8(rgb.r, _other.rgb.r, _frac);
        ret.rgb.g = lerp8by8(rgb.g, _other.rgb.g, _frac);
        ret.rgb.b = lerp8by8(rgb.b, _other.rgb.b, _frac);

        return ret;
    }

    /// @copydoc lerp8
    RGB_s lerp16(const RGB_s &_other, int _frac) const
    {
        RGB_s ret;

        ret.rgb.r = lerp16by16(rgb.r << 8, _other.rgb.r << 8, _frac) >> 8;
        ret.rgb.g = lerp16by16(rgb.g << 8, _other.rgb.g << 8, _frac) >> 8;
        ret.rgb.b = lerp16by16(rgb.b << 8, _other.rgb.b << 8, _frac) >> 8;

        return ret;
    }

    /// Returns 0 or 1, depending on the lowest bit of the sum of the color components.
    uint8_t getParity()
    {
        uint8_t sum = rgb.r + rgb.g + rgb.b;
        return (sum & 0x01);
    }

    /// Adjusts the color in the smallest way possible
    /// so that the parity of the coloris now the desired value.
    /// This allows you to "hide" one bit of information in the color.
    ///
    /// Ideally, we find one color channel which already
    /// has data in it, and modify just that channel by one.
    /// We don't want to light up a channel that's black
    /// if we can avoid it, and if the pixel is 'grayscale',
    /// (meaning that R==G==B), we modify all three channels
    /// at once, to preserve the neutral hue.
    ///
    /// There's no such thing as a free lunch; in many cases
    /// this "hidden bit" may actually be visible, but this
    /// code makes reasonable efforts to hide it as much
    /// as is reasonably possible.
    ///
    /// Also, an effort is made to make it such that
    /// repeatedly setting the parity to different values
    /// will not cause the color to "drift". Toggling
    /// the parity twice should generally result in the
    /// original color again.
    ///
    void setParity(uint8_t _parity)
    {
        uint8_t curparity = getParity();

        if (_parity == curparity)
            return;

        if (_parity) {
            // going 'up'
            if ((rgb.b > 0) && (rgb.b < 255)) {
                if (rgb.r == rgb.g && rgb.g == rgb.b) {
                    ++rgb.r;
                    ++rgb.g;
                }
                ++rgb.b;
            } else if ((rgb.r > 0) && (rgb.r < 255)) {
                ++rgb.r;
            } else if ((rgb.g > 0) && (rgb.g < 255)) {
                ++rgb.g;
            } else {
                if (rgb.r == rgb.g && rgb.g == rgb.b) {
                    rgb.r ^= 0x01;
                    rgb.g ^= 0x01;
                }
                rgb.b ^= 0x01;
            }
        } else {
            // going 'down'
            if (rgb.b > 1) {
                if (rgb.r == rgb.g && rgb.g == rgb.b) {
                    --rgb.r;
                    --rgb.g;
                }
                --rgb.b;
            } else if (rgb.g > 1) {
                --rgb.g;
            } else if (rgb.r > 1) {
                --rgb.r;
            } else {
                if (rgb.r == rgb.g && rgb.g == rgb.b) {
                    rgb.r ^= 0x01;
                    rgb.g ^= 0x01;
                }
                rgb.b ^= 0x01;
            }
        }
    }
};

/// Check if two RGB_s objects have the same color data
inline bool operator==(const RGB_s &_lhs, const RGB_s &_rhs)
{
    return (_lhs.rgb.r == _rhs.rgb.r) && (_lhs.rgb.g == _rhs.rgb.g) && (_lhs.rgb.b == _rhs.rgb.b);
}

/// Check if two RGB_s objects do *not* have the same color data
inline bool operator!=(const RGB_s &_lhs, const RGB_s &_rhs) { return !(_lhs == _rhs); }

/// Check if the sum of the color channels in one RGB_s object is less than another
inline bool operator<(const RGB_s &_lhs, const RGB_s &_rhs)
{
    uint16_t sl, sr;
    sl = _lhs.rgb.r + _lhs.rgb.g + _lhs.rgb.b;
    sr = _rhs.rgb.r + _rhs.rgb.g + _rhs.rgb.b;
    return sl < sr;
}

/// Check if the sum of the color channels in one RGB_s object is greater than another
inline bool operator>(const RGB_s &_lhs, const RGB_s &_rhs)
{
    uint16_t sl, sr;
    sl = _lhs.rgb.r + _lhs.rgb.g + _lhs.rgb.b;
    sr = _rhs.rgb.r + _rhs.rgb.g + _rhs.rgb.b;
    return sl > sr;
}

/// Check if the sum of the color channels in one RGB_s object is greater than or equal to another
inline bool operator>=(const RGB_s &_lhs, const RGB_s &_rhs)
{
    uint16_t sl, sr;
    sl = _lhs.rgb.r + _lhs.rgb.g + _lhs.rgb.b;
    sr = _rhs.rgb.r + _rhs.rgb.g + _rhs.rgb.b;
    return sl >= sr;
}

/// Check if the sum of the color channels in one RGB_s object is less than or equal to another
inline bool operator<=(const RGB_s &_lhs, const RGB_s &_rhs)
{
    uint16_t sl, sr;
    sl = _lhs.rgb.r + _lhs.rgb.g + _lhs.rgb.b;
    sr = _rhs.rgb.r + _rhs.rgb.g + _rhs.rgb.b;
    return sl <= sr;
}


/// @copydoc RGB_s::operator+=
inline RGB_s operator+(const RGB_s &_p1, const RGB_s &_p2)
{
    return RGB_s(qadd8(_p1.rgb.r, _p2.rgb.r), qadd8(_p1.rgb.g, _p2.rgb.g), qadd8(_p1.rgb.b, _p2.rgb.b));
}

/// @copydoc RGB_s::operator-=
inline RGB_s operator-(const RGB_s &_p1, const RGB_s &_p2)
{
    return RGB_s(qsub8(_p1.rgb.r, _p2.rgb.r), qsub8(_p1.rgb.g, _p2.rgb.g), qsub8(_p1.rgb.b, _p2.rgb.b));
}

/// @copydoc RGB_s::operator*=
inline RGB_s operator*(const RGB_s &_p1, uint8_t _d)
{
    return RGB_s(qmul8(_p1.rgb.r, _d), qmul8(_p1.rgb.g, _d), qmul8(_p1.rgb.b, _d));
}

/// @copydoc RGB_s::operator/=
inline RGB_s operator/(const RGB_s &_p1, uint8_t _d) { return RGB_s(_p1.rgb.r / _d, _p1.rgb.g / _d, _p1.rgb.b / _d); }


/// Combine two RGB_s objects, taking the smallest value of each channel
inline RGB_s operator&(const RGB_s &_p1, const RGB_s &_p2)
{
    return RGB_s(_p1.rgb.r < _p2.rgb.r ? _p1.rgb.r : _p2.rgb.r, _p1.rgb.g < _p2.rgb.g ? _p1.rgb.g : _p2.rgb.g,
                 _p1.rgb.b < _p2.rgb.b ? _p1.rgb.b : _p2.rgb.b);
}

/// Combine two RGB_s objects, taking the largest value of each channel
inline RGB_s operator|(const RGB_s &_p1, const RGB_s &_p2)
{
    return RGB_s(_p1.rgb.r > _p2.rgb.r ? _p1.rgb.r : _p2.rgb.r, _p1.rgb.g > _p2.rgb.g ? _p1.rgb.g : _p2.rgb.g,
                 _p1.rgb.b > _p2.rgb.b ? _p1.rgb.b : _p2.rgb.b);
}

/// Scale using RGB_s::nscale8_video()
inline RGB_s operator%(const RGB_s &_p1, uint8_t _d)
{
    RGB_s retval(_p1);
    retval.nscale8Video(_d);
    return retval;
}

/// RGB color channel orderings, used when instantiating controllers to determine
/// what order the controller should send data out in. The default ordering
/// is RGB.
/// Within this enum, the red channel is 0, the green channel is 1, and the
/// blue chanel is 2.
enum class EOrder_e : uint8_t {
    RGB = 0012, ///< Red,   Green, Blue  (0012)
    RBG = 0021, ///< Red,   Blue,  Green (0021)
    GRB = 0102, ///< Green, Red,   Blue  (0102)
    GBR = 0120, ///< Green, Blue,  Red   (0120)
    BRG = 0201, ///< Blue,  Red,   Green (0201)
    BGR = 0210  ///< Blue,  Green, Red   (0210)
}; // TODO: future - add WRGB variants

} // namespace LED
