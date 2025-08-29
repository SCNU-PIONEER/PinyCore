#pragma once

namespace FILTER {
class Mean {
public:
    float process(float _in)
    {
        sum_ += _in;
        count_++;
        return sum_ / static_cast<float>(count_);
    }

    void reset()
    {
        sum_ = 0.f;
        count_ = 0;
    }

private:
    float sum_ = 0.f;
    int count_ = 0;
};


class WeightedMean3 {
    static constexpr float COF[3] = { 0.2f, 0.6f, 0.2f };

public:
    float process(float _in)
    {
        float y = (COF[0] * _in) + (COF[1] * x_[0]) + (COF[2] * x_[1]);
        x_[1] = x_[0];
        x_[0] = _in;
        return y;
    }

private:
    float x_[2]{};
};

}; // namespace FILTER
