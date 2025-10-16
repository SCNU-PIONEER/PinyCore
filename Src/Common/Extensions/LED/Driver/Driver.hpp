#pragma once

#include "../Utils/Color.hpp"
#include <cstring>
#include <vector>

namespace LED {

class LEDDriver {
protected:
    friend class LEDs; // only LEDs can touch drivers
    using Color = RGB_s::Color_e;
    uint8_t vectorIndex_ = 0; // index in the LEDs manager color vector
    LEDDriver *nextDriver_ = nullptr;
    int numLEDs_ = 1; // number of LEDs in this node
    static LEDDriver *headDriver_;
    static LEDDriver *tailDriver_;

    virtual void show(std::vector<RGB_s> &_data) = 0;

public:
    virtual ~LEDDriver() = default;
    LEDDriver()
    {
        if (headDriver_ == nullptr) {
            headDriver_ = this;
        }
        if (tailDriver_ != nullptr) {
            tailDriver_->nextDriver_ = this;
        }
        tailDriver_ = this;
    }

    static LEDDriver *head() { return headDriver_; }
    LEDDriver *next() { return nextDriver_; }
    void setIndex(uint8_t _index) { vectorIndex_ = _index; }

    /// How many LEDs does this controller manage?
    /// @returns LEDDriver::numLEDs_
    virtual int size() { return numLEDs_; }
};

} // namespace LED
