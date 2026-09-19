#pragma once

#include <cstddef>
#include <cstdint>

namespace LOG {

class Output {
public:
    virtual ~Output() = default;

    virtual bool init() = 0;
    virtual bool send(const uint8_t *_data, size_t _size) = 0;
};

} // namespace LOG
