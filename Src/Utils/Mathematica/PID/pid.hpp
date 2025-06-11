#pragma once

class PID {
public:
    virtual float calc(float ref, float cur) = 0;
    virtual void reset() = 0;

    template <typename T> T limitMinMax(T x, T min_val, T max_val)
    {
        return (min_val < max_val) ?
                       (x <= min_val ? min_val : (x >= max_val ? max_val : x)) :
                       (x <= max_val ? max_val : (x >= min_val ? min_val : x));
    }
};
