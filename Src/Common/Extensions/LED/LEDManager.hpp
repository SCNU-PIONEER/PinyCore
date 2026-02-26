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

    /**
     * @brief Adds a new driver to the LED manager system
     * 
     * @param _driver Pointer to the LED driver instance to be added
     * @param _colorData Initial color data array for the driver
     * 
     * @note The manager takes ownership of the driver instance
     * @note The color data array must match the driver's LED count
     */
    void addLEDs(LEDDriver *_driver);

    /// Show the current LED colors
    void show();

    /**
     * @brief Creates a driver instance for the specified PWM chipset type
     * 
     * @tparam Chip The chipset type to create (template parameter)
     * @param _handle Handle to the PWM peripheral for LED control
     * @param _num Total number of LEDs to be controlled by the driver
     * 
     * @return LEDDriver* Pointer to the created driver instance
     */
    template <PWMChipsets_e Chip> LEDDriver *create(Pwm *_handle, int _num)
    {
        switch (Chip) {
        case PWM_WS2812B:
            return new WS2812PWMDriver(_handle, _num);
        default:
            return nullptr;
        }
    }

    /**
    * @brief Creates a driver instance for the specified SPI chipset type
    * 
    * @tparam Chip The chipset type to create (template parameter)
    * @param _handle Handle to the SPI peripheral for communication
    * @param _num Total number of LEDs to be controlled by the driver
    * 
    * @return LEDDriver* Pointer to the created driver instance
    */
    template <SPIChipsets_e Chip> LEDDriver *create(SPI_HandleTypeDef *_handle, int _num)
    {
        switch (Chip) {
        case SPI_WS2812B:
            return new WS2812SPIDriver(_handle, _num);
        default:
            return nullptr;
        }
    }

    /**
     * @brief Creates a driver instance for the specified GPIO chipset type
     * 
     * @tparam Chip The chipset type to create (template parameter)
     * @param _setR Function pointer to set the red value of LEDs
     * @param _setG Function pointer to set the green value of LEDs
     * @param _setB Function pointer to set the blue value of LEDs
     * @param _num Total number of LEDs to be controlled by the driver
     * 
     * @return LEDDriver* Pointer to the created driver instance
     */
    template <IOChipsets_e Chip>
    static LEDDriver *create(RGBLEDDriver::lightTuner _setR(uint8_t), RGBLEDDriver::lightTuner _setG(uint8_t),
                             RGBLEDDriver::lightTuner _setB(uint8_t))
    {
        switch (Chip) {
        case RGBLED:
            return new RGBLEDDriver(_setR, _setG, _setB);
        default:
            return nullptr;
        }
    }

    //-----------------------------------------------------------------------------------
    /**
     * @brief Controls the LEDs with specified parameters
     * 
     * @param _type The type of command to execute
     * @param _index The index of the totalLEDs_ array
     * @param _ctrlNum The number of LEDs to control starting from _index position
     * 
     * @note This function modifies the LED states based on the command type and parameters
     * 
     * @warning Ensure _index is within bounds of totalLEDs_ array
     * @warning Ensure _ctrlNum does not exceed array bounds starting from _index
     */
    static void ctrl(CmdType_e _type, uint8_t _index, uint8_t _ctrlNum);

    /**
     * @brief Turn off all LEDs
     */
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
