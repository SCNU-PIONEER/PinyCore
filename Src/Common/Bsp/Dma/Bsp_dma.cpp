#include "Bsp_dma.hpp"
#include <stdlib.h>


void *Dma::ram_alloc(size_t _size)
{
    constexpr size_t alignment = 4;
    _size = (_size + alignment - 1) & ~(alignment - 1); // 统一处理对齐

#ifdef SOC_DMA_RAM
    void *ptr = aligned_alloc(4, _size); // 分配对齐内存
    return ptr;
#endif
#ifdef SOC_DMA_SRAM
    uint8_t *_ramDmaStart = (uint8_t *)SOC_DMA_SRAM;
    uint8_t *_ramDmaEnd = (uint8_t *)SOC_DMA_SRAM_END;
    static uint8_t *dma_heap_ptr = (uint8_t *)_ramDmaStart;
    uint8_t *ptr = NULL;
    if ((_ramDmaStart + _size) <= (uint8_t *)_ramDmaEnd) {
        ptr = dma_heap_ptr;
        dma_heap_ptr += _size;
    }
    return ptr;
#endif
}

void *Dma::ram_alloc(size_t _size, DmaRam_e _ram)
{
    constexpr size_t alignment = 4;
    _size = (_size + alignment - 1) & ~(alignment - 1); // 统一处理对齐

    switch (_ram) {
#ifdef SOC_DMA_RAM
    case DmaRam_e::RAM: {
        void *ptr = aligned_alloc(4, _size); // 分配对齐内存
        return ptr;
    }
#endif
    break;
#ifdef SOC_DMA_SRAM
    case DmaRam_e::SRAM: {
        uint8_t *_ramDmaStart = (uint8_t *)SOC_DMA_SRAM;
        uint8_t *_ramDmaEnd = (uint8_t *)SOC_DMA_SRAM_END;
        static uint8_t *dma_heap_ptr = (uint8_t *)_ramDmaStart;
        uint8_t *ptr = NULL;
        if ((_ramDmaStart + _size) <= (uint8_t *)_ramDmaEnd) {
            ptr = dma_heap_ptr;
            dma_heap_ptr += _size;
        }
        return ptr;
    }
#endif
    break;
#ifdef SOC_DMA_SRAM_D2
    case DmaRam_e::SRAM_D2: {
        uint8_t *_ramD2DmaStart = (uint8_t *)SOC_DMA_SRAM_D2;
        uint8_t *_ramD2DmaEnd = (uint8_t *)SOC_DMA_SRAM_D2_END;
        static uint8_t *_dmaD2HeapPtr = (uint8_t *)_ramD2DmaStart;
        uint8_t *D2ptr = NULL;
        if ((_ramD2DmaStart + _size) <= (uint8_t *)_ramD2DmaEnd) {
            D2ptr = _dmaD2HeapPtr;
            _dmaD2HeapPtr += _size;
        }
        return D2ptr;
    }
#endif
    break;
    }
    return nullptr;
}

void Dma::ram_free(void *_ptr) { free(_ptr); }
