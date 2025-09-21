#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>

#include "CaliParamsDefs.hpp"

#define CORRECT_IMU_DATA 1

namespace INS_SYS {

typedef struct {
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
} CaliOutput_s;

class IMUCalibration {
    static constexpr float BIAS_ALPHA = 0.007f;
    static constexpr uint16_t STEADY_CNT_MAX = 10;
    static constexpr float STEADY_ACCEL_RANGE = 0.28f;
    static constexpr float STEADY_GYRO_RANGE = 0.03f;
    static constexpr float GYRO_BIAS_MAX_RAW = 100.f;

    static constexpr float G =
            9.7883f; // (GuangZhou) Default gravity constant in m/s^2

public:
    void init(const AccCaliParams_s &_accCali,
              const GyroCaliParams_s &_gyroCali, float _aTransK = 1.0f,
              float _gTransK = 1.0f);

    void correctA(int16_t _ax, int16_t _ay, int16_t _az);
    void correctG(int16_t _gx, int16_t _gy, int16_t _gz);
    void correctM(int16_t _mx, int16_t _my, int16_t _mz);

    const CaliOutput_s &getOutput() const { return corrDat_; }

    // Detect steady state for gyro calibration
    void steadyStateDetection();

    void updateTemperature(float _temp) { temperature_ = _temp; }

private:
    AccCaliParams_s acc_cali_;
    GyroCaliParams_s gyro_cali_;

    // steady state detection parameters
    uint16_t staticSteadyStateCnt_ = 0;

    CaliOutput_s corrDat_; // calibrated output

    float temperature_ = 0.0f; // temperature data , unit:degC

    float aTransK_ = 1.0f;
    float gTransK_ = 1.0f;
};

} // namespace INS_SYS
