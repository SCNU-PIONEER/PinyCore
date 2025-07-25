#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>

#define GYRO_BIAS_MAX      (1.0f * 0.01745f)

#define BIAS_ALPHA         0.007f

#define STEADY_CNT_MAX     10

#define STEADY_ACCEL_RANGE 0.28f

#define STEADY_GYRO_RANGE  0.03f

#define GYRO_BIAS_MAX_RAW  100.f

typedef struct {
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
} imu_data_fp_t;

typedef struct {
    uint16_t gx, gy, gz;
    uint16_t ax, ay, az;
    float temperature;
} imu_data_raw_t;

typedef struct {
    float accel_T[3][3];
    float accel_offs[3];
} AccCali_s;

typedef struct {
    float gx_bias, gy_bias, gz_bias;
    float gx_tco_k, gx_tco_b0;
    float gy_tco_k, gy_tco_b0;
    float gz_tco_k, gz_tco_b0;
} GyroCali_s;

typedef enum : uint8_t {
    ACC_PX = 0,
    ACC_NX = 1,
    ACC_PX_O = 6,
    ACC_NX_O = 7,
    ACC_PY = 2,
    ACC_NY = 3,
    ACC_PY_O = 8,
    ACC_NY_O = 9,
    ACC_PZ = 4,
    ACC_NZ = 5,
    ACC_PZ_O = 10,
    ACC_NZ_O = 11,
    ACC_CAL_FINAL = 12
} acc_cali_state_t;


class IMUCalibration {
public:
    void init(const AccCali_s &_accCali, const GyroCali_s &_gyroCali,
              const float _temp = 0.0f);

    imu_data_fp_t correct(float _aTransK, float _gTransK, float _gx, float _gy,
                          float _gz, float _ax, float _ay, float _az,
                          float _temperature);
    imu_data_fp_t correctInt16(float _aTransK, float _gTransK, int16_t _gx,
                               int16_t _gy, int16_t _gz, int16_t _ax,
                               int16_t _ay, int16_t _az, float _temperature);

    // Detect steady state for gyro calibration
    imu_data_fp_t steadyStateDetection();

private:
    template <typename T> T clamp(T _value, T _max)
    {
        return std::max(-_max, std::min(_value, _max));
    }

    float invSqrt(float _x);

    AccCali_s acc_cali_;
    GyroCali_s gyro_cali_;

    float g_ = 9.80665f; // Default gravity constant in m/s^2

    // steady state detection parameters
    uint16_t staticSteadyStateCnt_ = 0;

    imu_data_fp_t corrDat_; // calibrated output
};
