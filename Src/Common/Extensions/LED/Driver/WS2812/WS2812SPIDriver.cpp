#include "WS2812SPIDriver.hpp"
#include "Bsp_dma.hpp"

#include <array>
#include <cstdint>
#include <cstring>

using namespace LED;

namespace {

using EncodedByte = std::array<uint8_t, 8>;
constexpr size_t BYTES_PER_LED = 24U;
constexpr size_t MAX_SPI_DMA_BYTES = 0xFFFFU;
constexpr uint8_t CODE1_HALF = 0x78U; // (0xF0 >> 1)
constexpr uint8_t CODE0_HALF = 0x60U; // (0xC0 >> 1)

const EncodedByte &encodedByteLut(uint8_t _value)
{
    static const auto LUT = [] {
        std::array<EncodedByte, 256> table{};
        for (uint16_t v = 0; v < 256; ++v) {
            const uint8_t b = static_cast<uint8_t>(v);
            for (uint8_t i = 0; i < 8; ++i) {
                table[static_cast<size_t>(v)][static_cast<size_t>(i)] = ((b & (1U << (7 - i))) != 0U) ? CODE1_HALF :
                                                                                                        CODE0_HALF;
            }
        }
        return table;
    }();

    return LUT[_value];
}

inline void writeEncodedByte(uint8_t *_dst, const EncodedByte &_encoded)
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

bool isBDMADma(const DMA_HandleTypeDef *_dma)
{
#if defined(BDMA_Channel0) && defined(BDMA_Channel7)
    if (_dma == nullptr || _dma->Instance == nullptr) {
        return false;
    }

    const auto instance = reinterpret_cast<uintptr_t>(_dma->Instance);
    return instance >= reinterpret_cast<uintptr_t>(BDMA_Channel0) &&
           instance <= reinterpret_cast<uintptr_t>(BDMA_Channel7);
#else
    (void)_dma;
    return false;
#endif
}

} // namespace

WS2812SPIDriver::WS2812SPIDriver(SPI_HandleTypeDef *_spiHandle, int _num) : LEDDriver(_num), spiHandle_(_spiHandle)
{
    if (spiHandle_ == nullptr) {
        return;
    }

    const size_t ledCount = (_num > 0) ? static_cast<size_t>(_num) : 0U;
    const size_t txLen = ledCount * BYTES_PER_LED;
    if (txLen == 0U || txLen > MAX_SPI_DMA_BYTES) {
        return;
    }

#ifdef SOC_DMA_SRAM_BDMA
    if (isBDMADma(spiHandle_->hdmatx)) {
        txbuf_ = static_cast<uint8_t *>(Dma::instance().ram_alloc(txLen, DmaRam_e::SRAM_BDMA));
    } else
#endif
#ifdef SOC_DMA_SRAM
    {
        txbuf_ = static_cast<uint8_t *>(Dma::instance().ram_alloc(txLen, DmaRam_e::SRAM));
    }
#else
    {
        txbuf_ = static_cast<uint8_t *>(Dma::instance().ram_alloc(txLen));
    }
#endif
    if (txbuf_ != nullptr) {
        txbufLen_ = static_cast<uint16_t>(txLen);
        std::memset(txbuf_, 0, txLen);
    }
}

void WS2812SPIDriver::show(std::vector<RGB_s> &_data)
{
    if (txbuf_ == nullptr || txbufLen_ == 0U) {
        return;
    }

    if (spiHandle_ == nullptr || spiHandle_->State != HAL_SPI_STATE_READY) {
        return;
    }

    RGB_s *ledData = &_data[vectorIndex_];

    for (uint16_t id = 0; id < static_cast<uint16_t>(numLEDs_); ++id) {
        uint8_t *dst = txbuf_ + (id * BYTES_PER_LED);
        const RGB_s &px = ledData[id];

        writeEncodedByte(dst, encodedByteLut(px.rgb.g));
        writeEncodedByte(dst + 8, encodedByteLut(px.rgb.r));
        writeEncodedByte(dst + 16, encodedByteLut(px.rgb.b));
    }
    Spi::instance().transmitDMA(spiHandle_, txbuf_, txbufLen_);
}
