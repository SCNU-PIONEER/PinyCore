#pragma once

#include "Singleton.hpp"
#include "Soc.hpp"
#include HAL_INCLUDE

class Dma : public Singleton<Dma> {
public:
    /**
     * @brief dma ram auto alloc 
     */
    void *ram_alloc(size_t size);

    /**
     * @brief dma ram alloc 
     */
    void *ram_alloc(size_t size, DmaRam_e _ram);

    /**
     * @brief dma ram free 
     */
    void ram_free(void *_ptr);

private:
    Dma() = default;
    friend class Singleton<Dma>;
};
