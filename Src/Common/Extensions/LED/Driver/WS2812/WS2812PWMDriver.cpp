#include "WS2812PWMDriver.hpp"
#include "Bsp_dma.hpp"

#include <array>
#include <cstring>

using namespace LED;

namespace {

using EncodedByte = std::array<uint8_t, 8>;
constexpr uint8_t CODE1 = 0x86U;
constexpr uint8_t CODE0 = 0x43U;

const EncodedByte &encodedByteLut(uint8_t _value)
{
    static const auto LUT = [] {
        std::array<EncodedByte, 256> table{};
        for (uint16_t v = 0; v < 256; ++v) {
            const uint8_t b = static_cast<uint8_t>(v);
            for (uint8_t i = 0; i < 8; ++i) {
                table[static_cast<size_t>(v)][static_cast<size_t>(i)] = ((b & (1U << (7 - i))) != 0U) ? CODE1 : CODE0;
            }
        }
        return table;
    }();

    return LUT[_value];
}

inline void writeEncodedByte(uint32_t *_dst, const EncodedByte &_encoded)
{
    _dst[0] = _encoded[0];
    _dst[1] = _encoded[1];
    _dst[2] = _encoded[2];
    _dst[3] = _encoded[3];
    _dst[4] = _encoded[4];
    _dst[5] = _encoded[5];
    _dst[6] = _encoded[6];
    _dst[7] = _encoded[7];
}

} // namespace

WS2812PWMDriver::WS2812PWMDriver(Pwm *_pwmHandle, int _num)
        : LEDDriver(_num)
        , pwm_(_pwmHandle)
        , txbuf(static_cast<uint32_t *>(Dma::instance().ram_alloc((numLEDs_ + 1) * 24 * sizeof(uint32_t))))
{
    std::memset(txbuf, 0, (numLEDs_ + 1) * 24 * sizeof(uint32_t));
}

void WS2812PWMDriver::show(std::vector<RGB_s> &_data)
{
    RGB_s *ledData = &_data[vectorIndex_];

    for (uint16_t id = 0; id < static_cast<uint16_t>(numLEDs_); ++id) {
        uint32_t *dst = txbuf + (id * 24);
        const RGB_s &px = ledData[id];

        writeEncodedByte(dst, encodedByteLut(px.rgb.g));
        writeEncodedByte(dst + 8, encodedByteLut(px.rgb.r));
        writeEncodedByte(dst + 16, encodedByteLut(px.rgb.b));
    }

    pwm_->startDMA(txbuf, (numLEDs_ + 1) * 24);
}
