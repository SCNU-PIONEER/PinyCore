#pragma once

class PID {
public:
    virtual ~PID() = default;

    virtual float calc(float ref, float cur) = 0;
    virtual void reset() = 0;
};
