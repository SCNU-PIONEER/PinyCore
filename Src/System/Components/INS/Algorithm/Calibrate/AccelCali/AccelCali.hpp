#pragma once

#include <cstdint>

#include "../CaliParamsDefs.hpp"
#include <utility>

namespace INS_SYS {

class AccelCali {
    static constexpr float G = 9.7833f;
    static constexpr uint8_t STATIONART_DETECTION_BUFFER_SIZE = 100;

    enum class Orientation_e : uint8_t {
        UNKOWN = 0u,
        X_UP = 1u,
        X_DOWN = 2u,
        Y_UP = 3u,
        Y_DOWN = 4u,
        Z_UP = 5u,
        Z_DOWN = 6u
    };

    constexpr static const std::pair<uint8_t, const char *> ORIENTATION_STR[] = {
        { static_cast<uint8_t>(Orientation_e::UNKOWN), "UNKNOWN" },
        { static_cast<uint8_t>(Orientation_e::X_UP), "X_UP" },
        { static_cast<uint8_t>(Orientation_e::X_DOWN), "X_DOWN" },
        { static_cast<uint8_t>(Orientation_e::Y_UP), "Y_UP" },
        { static_cast<uint8_t>(Orientation_e::Y_DOWN), "Y_DOWN" },
        { static_cast<uint8_t>(Orientation_e::Z_UP), "Z_UP" },
        { static_cast<uint8_t>(Orientation_e::Z_DOWN), "Z_DOWN" }
    };

public:
    void update(int16_t _ax, int16_t _ay, int16_t _az, int16_t _gx, int16_t _gy,
                int16_t _gz, float _aTransK, float _gTransK);

    AccelCali(const AccelCali &) = delete;
    AccelCali &operator=(const AccelCali &) = delete;

    static AccelCali &instance();

private:
    AccelCali();

    void startCalibration();

    void calculateCalibrationParams();
    void saveCalibrationParams();

    void handleIdle();
    void handleWaitForCollecting();

    bool isCorrectOrientation();
    bool isStationary();
    bool collectData();

    AccCaliParams_s caliParams_;

    uint8_t side_ = 0;

    float refData_[6][3]{}; // 6 sides, 3 axis
    uint16_t collectCnt_ = 0;

    struct StationaryData_s {
        float accelSamples[STATIONART_DETECTION_BUFFER_SIZE][3];
        float gyroSamples[STATIONART_DETECTION_BUFFER_SIZE][3];
        uint8_t index = 0;
        bool isFull = false;
    } stationaryData_{};

    struct IMUDat_s {
        float ax, ay, az;
        float gx, gy, gz;
    } imuData_{};
};

}; // namespace INS_SYS
