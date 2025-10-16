#pragma once

#include "../Utils/Color.hpp"
#include <cstring>
#include <memory>

namespace LED {

class LEDDriver {
protected:
    friend class LEDs; // only LEDs can touch drivers
    using Color = RGB_s::Color_e;
    RGB_s *colorData_ = nullptr;
    LEDDriver *nextDriver_ = nullptr;
    int numLEDs_ = 1; // number of LEDs in this node
    static LEDDriver *headDriver_;
    static LEDDriver *tailDriver_;

    virtual void show() = 0;

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

    void setColor(Color _color, int _index)
    {
        uint32_t c = static_cast<uint32_t>(_color);
        colorData_[_index].setColorCode(c);
    }

    virtual void init() { clearColor(numLEDs_); }

    virtual void clearColor(int _numLEDs)
    {
        for (int i = 0; i < _numLEDs; ++i)
            setColor(Color::Black, i);
    }

    static LEDDriver *head() { return headDriver_; }
    LEDDriver *next() { return nextDriver_; }
    LEDDriver &createLEDs(int _numLEDs)
    {
        auto colorData = std::make_unique<RGB_s[]>(_numLEDs);
        colorData_ = colorData.get();

        numLEDs_ = _numLEDs;
        return *this;
    }

    /// How many LEDs does this controller manage?
    /// @returns LEDDriver::numLEDs_
    virtual int size() { return numLEDs_; }

    /// Pointer to the CRGB array for this controller
    /// @returns LEDDriver::colorData_
    RGB_s *leds() { return colorData_; }

    /// Reference to the n'th LED managed by the controller
    /// @param x the LED number to retrieve
    /// @returns reference to LEDDriver::colorData_[x]
    RGB_s &operator[](int _x) { return colorData_[_x]; }
};

} // namespace LED
