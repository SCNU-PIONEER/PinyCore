#pragma once

#include "Driver/WS2812/WS2812SPIDriver.hpp"
#include "Driver/WS2812/WS2812PWMDriver.hpp"
#include "Driver/RGBLED/RGBLEDDriver.hpp"

#include "FreeRTOS.h"
#include "queue.h"

#include "CmdType.h"
#include <vector>
#include <memory>

namespace LED {

enum SPIChipsets_e : uint8_t {
    SPI_WS2812B = 0,
};

enum PWMChipsets_e : uint8_t {
    PWM_WS2812B = 0,
};

enum IOChipsets_e : uint8_t {
    RGBLED = 0,
};

class LEDs {
public:
    static LEDs &instance();
    LEDs(const LEDs &) = delete;
    LEDs &operator=(const LEDs &) = delete;

    static void task(void *_param);

    /// Add a driver to the LED manager
    /// @param _driver the driver to add
    /// @param _colorData the color data for the driver
    /// @return true if the driver was added successfully, false otherwise
    void addLEDs(LEDDriver *_driver);

    /// Show the current LED colors
    void show();

    /// Create a driver of the PWM chipset type
    /// @tparam Chip the chipset type to create
    /// @param _handle the handle to the chipset
    /// @param _num the number of LEDs in the driver
    /// @return a pointer to the created driver
    template <PWMChipsets_e Chip> LEDDriver *create(Pwm *_handle, int _num)
    {
        switch (Chip) {
        case PWM_WS2812B:
            return std::make_unique<WS2812PWMDriver>(_handle, _num).release();
        default:
            return nullptr;
        }
    }

    /// Create a driver of the given chipset type
    /// @tparam Chip the chipset type to create
    /// @param _setR the function to set the red value
    /// @param _setG the function to set the green value
    /// @param _setB the function to set the blue value
    /// @param _num the number of LEDs in the driver
    /// @return a pointer to the created driver
    template <IOChipsets_e Chip>
    static LEDDriver *create(RGBLEDDriver::lightTuner _setR(uint8_t),
                             RGBLEDDriver::lightTuner _setG(uint8_t),
                             RGBLEDDriver::lightTuner _setB(uint8_t))
    {
        switch (Chip) {
        case RGBLED:
            return std::make_unique<RGBLEDDriver>(_setR, _setG, _setB).release();
        default:
            return nullptr;
        }
    }

    //-----------------------------------------------------------------------------------
    static void ctrl(CmdType_e _type, uint8_t _index, uint8_t _ctrlNum);
    static void off();

private:
    LEDs();

    QueueHandle_t queue_ = nullptr;

    std::vector<RGB_s> ledColors_;

    int totalLEDs_ = 0; // total number of LEDs across all drivers

    // heart beat
    uint32_t lastTime_ = 0;

    void handleOff(uint8_t _index, uint8_t _ctrlNum);
    void handleOnInNormal(uint8_t _index, uint8_t _ctrlNum);
    void handleBlinkRGB(uint8_t _index, uint8_t _ctrlNum);

    void handleRainbowFlow(uint8_t _index, uint8_t _ctrlNum);
};

} // namespace LED
