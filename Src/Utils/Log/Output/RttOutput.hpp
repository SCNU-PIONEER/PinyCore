#pragma once

#include "Output.hpp"

namespace LOG {

class RttOutput final : public Output {
public:
    bool init() final;
    bool send(const uint8_t *_data, size_t _size) final;
};

} // namespace LOG
