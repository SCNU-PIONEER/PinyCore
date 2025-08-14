#pragma once

class PID {
public:
    virtual ~PID() = default;

    virtual float calc(float _ref, float _cur) = 0;
    virtual void reset() = 0;
};
