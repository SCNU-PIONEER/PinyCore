#include "WS2812PWMDriver.hpp"
#include "Bsp_dma.hpp"

#include <cstring>

using namespace LED;

namespace {

constexpr uint32_t REFERENCE_PERIOD_TICKS = 210U;
constexpr uint32_t REFERENCE_CODE1 = 0x86U;
constexpr uint32_t REFERENCE_CODE0 = 0x43U;

uint32_t scaledCode(uint32_t _periodTicks, uint32_t _referenceCode)
{
    if (_periodTicks == 0U) {
        return 0U;
    }

    const uint32_t code = (_periodTicks * _referenceCode + REFERENCE_PERIOD_TICKS / 2U) / REFERENCE_PERIOD_TICKS;
    return code < _periodTicks ? code : _periodTicks - 1U;
}

inline void writeEncodedByte(uint32_t *_dst, const std::array<uint32_t, 8> &_encoded)
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

WS2812PWMDriver::WS2812PWMDriver(TIM_HandleTypeDef *_timer, uint32_t _channel, int _num)
        : LEDDriver(_num)
        , pwm_(_timer, _channel)
        , txbuf(static_cast<uint32_t *>(Dma::instance().ram_alloc((numLEDs_ + 1) * 24 * sizeof(uint32_t))))
{
    std::memset(txbuf, 0, (numLEDs_ + 1) * 24 * sizeof(uint32_t));
    updateEncodedByteLut();
}

void WS2812PWMDriver::updateEncodedByteLut()
{
    const uint32_t periodTicks = pwm_.autoLoader() + 1U;
    if (periodTicks == periodTicks_) {
        return;
    }

    periodTicks_ = periodTicks;
    const uint32_t code0 = scaledCode(periodTicks_, REFERENCE_CODE0);
    const uint32_t code1 = scaledCode(periodTicks_, REFERENCE_CODE1);

    for (uint16_t v = 0; v < 256; ++v) {
        const uint8_t b = static_cast<uint8_t>(v);
        EncodedByte &encoded = encodedByteLut_[static_cast<size_t>(v)];
        encoded[0] = ((b & 0x80U) != 0U) ? code1 : code0;
        encoded[1] = ((b & 0x40U) != 0U) ? code1 : code0;
        encoded[2] = ((b & 0x20U) != 0U) ? code1 : code0;
        encoded[3] = ((b & 0x10U) != 0U) ? code1 : code0;
        encoded[4] = ((b & 0x08U) != 0U) ? code1 : code0;
        encoded[5] = ((b & 0x04U) != 0U) ? code1 : code0;
        encoded[6] = ((b & 0x02U) != 0U) ? code1 : code0;
        encoded[7] = ((b & 0x01U) != 0U) ? code1 : code0;
    }
}

void WS2812PWMDriver::show(std::vector<RGB_s> &_data)
{
    if (!pwm_.isReady()) {
        return;
    }

    RGB_s *ledData = &_data[vectorIndex_];
    updateEncodedByteLut();

    for (uint16_t id = 0; id < static_cast<uint16_t>(numLEDs_); ++id) {
        uint32_t *dst = txbuf + (id * 24);
        const RGB_s &px = ledData[id];

        writeEncodedByte(dst, encodedByteLut_[px.rgb.g]);
        writeEncodedByte(dst + 8, encodedByteLut_[px.rgb.r]);
        writeEncodedByte(dst + 16, encodedByteLut_[px.rgb.b]);
    }

    pwm_.startDMA(txbuf, (numLEDs_ + 1) * 24);
}
