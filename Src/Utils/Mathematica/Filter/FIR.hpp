#pragma once

namespace FILTER {

class FIR5 {
    static constexpr float NUM_5[6] = { 0.04964470491f, 0.1659367979f,
                                        0.2844184935f,  0.2844184935f,
                                        0.1659367979f,  0.04964470491f };

public:
    float process(float _in)
    {
        float out = (_in * NUM_5[0]) + (x_[0] * NUM_5[1]) + (x_[1] * NUM_5[2]) +
                    (x_[2] * NUM_5[3]) + (x_[3] * NUM_5[4]) +
                    (x_[4] * NUM_5[5]);
        x_[4] = x_[3];
        x_[3] = x_[2];
        x_[2] = x_[1];
        x_[1] = x_[0];
        x_[0] = _in;
        return out;
    }

private:
    float x_[5]{};
};

} // namespace FILTER
