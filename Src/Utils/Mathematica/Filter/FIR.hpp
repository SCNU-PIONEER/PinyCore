#pragma once
#include <cstdint>
namespace FILTER {
template <uint8_t RANK> class FIR {
public:
    FIR(float _num[RANK + 1]) { memcpy(NUM_, _num, sizeof(NUM_)); }
    float process(float _in)
    {
        float out = _in * NUM_[0];
        for (uint8_t i = 0; i < RANK; i++)
            out += x_[i] * NUM_[i + 1];
        for (uint8_t i = RANK - 1; i > 0; i--)
            x_[i] = x_[i - 1];
        x_[0] = _in;
        return out;
    }

private:
    float x_[RANK]{};
    float NUM_[RANK + 1] = {};
};
} // namespace FILTER
