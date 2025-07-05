#pragma once

#include "BspBase.hpp"
#include "Soc.hpp"
#include HAL_INCLUDE

class Dma : public BspBase<Dma> {
public:
    /**
     * @brief dma ram auto alloc 
     */
    void *ram_alloc(size_t size);

    /**
     * @brief dma ram alloc 
     */
    void *ram_alloc(size_t size, DmaRam_e _ram);
};
