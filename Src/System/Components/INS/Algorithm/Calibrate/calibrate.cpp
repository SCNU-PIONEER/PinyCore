#include <math.h>
#include <stdlib.h>
#include "calibrate.hpp"

#define IMU_RAW_RANGE (32768)

void ImuCalibration::init(const AccCali_s &accCali, const GyroCali_s &gyroCali,
                          const float _temp)
{
    acc_cali_ = accCali;
    gyro_cali_ = gyroCali;
    staticSteadyStateCnt_ = 0;
}
imu_data_fp_t
ImuCalibration::Correct(float _aFullscale, float _gFullscale_, float _aRawRange,
                        float _gRawRange, float _gx, float _gy, float _gz,
                        float _ax, float _ay, float _az, float _temperature)
{
    float g_k = _gFullscale_ * M_PI / 180.f / _gRawRange;
    float a_k = _aFullscale * g_ / _aRawRange;
    // de_ai or de_gi mean the raw decoded data to affinite Calibrate Algorithm
    // see the next function 'CorrectInt16' to understand this abstraction
    float de_ax = _ax / a_k, de_ay = _ay / a_k, de_az = _az / a_k;
    float de_gx = _gx / g_k, de_gy = _gy / g_k, de_gz = _gz / g_k;
    float ax_ub = de_ax - acc_cali_.accel_offs[0];
    float ay_ub = de_ay - acc_cali_.accel_offs[1];
    float az_ub = de_az - acc_cali_.accel_offs[2];
    corrDat_.ax =
            (acc_cali_.accel_T[0][0] * ax_ub + acc_cali_.accel_T[0][1] * ay_ub +
             acc_cali_.accel_T[0][2] * az_ub) * a_k;
    corrDat_.ay =
            (acc_cali_.accel_T[1][0] * ax_ub + acc_cali_.accel_T[1][1] * ay_ub +
             acc_cali_.accel_T[1][2] * az_ub) * a_k;
    corrDat_.az =
            (acc_cali_.accel_T[2][0] * ax_ub + acc_cali_.accel_T[2][1] * ay_ub +
             acc_cali_.accel_T[2][2] * az_ub) * a_k;

    corrDat_.gx =
            (de_gx - gyro_cali_.gx_bias -
             (gyro_cali_.gx_tco_k * _temperature + gyro_cali_.gx_tco_b0)) *
            g_k;
    corrDat_.gy =
            (de_gy - gyro_cali_.gy_bias -
             (gyro_cali_.gy_tco_k * _temperature + gyro_cali_.gy_tco_b0)) *
            g_k;
    corrDat_.gz =
            (de_gz - gyro_cali_.gz_bias -
             (gyro_cali_.gz_tco_k * _temperature + gyro_cali_.gz_tco_b0)) *
            g_k;
    
    return corrDat_;
}
imu_data_fp_t
ImuCalibration::CorrectInt16(float _aFullscale, float _gFullscale_,
                             float _aRawRange, float _gRawRange, uint16_t _gx,
                             uint16_t _gy, uint16_t _gz, uint16_t _ax,
                             uint16_t _ay, uint16_t _az, float _temperature)
{
    float g_k = _gFullscale_ * M_PI / 180.f / _gRawRange;
    float a_k = _aFullscale * g_ / _aRawRange;
    float ax_ub = _ax - acc_cali_.accel_offs[0];
    float ay_ub = _ay - acc_cali_.accel_offs[1];
    float az_ub = _az - acc_cali_.accel_offs[2];
    corrDat_.ax =
            (acc_cali_.accel_T[0][0] * ax_ub + acc_cali_.accel_T[0][1] * ay_ub +
             acc_cali_.accel_T[0][2] * az_ub) * a_k;
    corrDat_.ay =
            (acc_cali_.accel_T[1][0] * ax_ub + acc_cali_.accel_T[1][1] * ay_ub +
             acc_cali_.accel_T[1][2] * az_ub) * a_k;
    corrDat_.az =
            (acc_cali_.accel_T[2][0] * ax_ub + acc_cali_.accel_T[2][1] * ay_ub +
             acc_cali_.accel_T[2][2] * az_ub) * a_k;

    corrDat_.gx =
            (((float)_gx) - gyro_cali_.gx_bias -
             (gyro_cali_.gx_tco_k * _temperature + gyro_cali_.gx_tco_b0)) *
            g_k;
    corrDat_.gy =
            (((float)_gy) - gyro_cali_.gy_bias -
             (gyro_cali_.gy_tco_k * _temperature + gyro_cali_.gy_tco_b0)) *
            g_k;
    corrDat_.gz =
            (((float)_gz) - gyro_cali_.gz_bias -
             (gyro_cali_.gz_tco_k * _temperature + gyro_cali_.gz_tco_b0)) *
            g_k;
    
    return corrDat_;
}

imu_data_fp_t ImuCalibration::steadyStateDetection()
{
    float recipNorm =
            invSqrt(corrDat_.ax * corrDat_.ax + corrDat_.ay * corrDat_.ay +
                    corrDat_.az * corrDat_.az);
    if (((g_ * recipNorm) < (1 + STEADY_ACCEL_RANGE)) &&
        ((g_ * recipNorm) > (1 - STEADY_ACCEL_RANGE)) &&
        fabsf(corrDat_.gx) < STEADY_GYRO_RANGE && fabsf(corrDat_.gy) < STEADY_GYRO_RANGE &&
        fabsf(corrDat_.gz) < STEADY_GYRO_RANGE) {
        if (staticSteadyStateCnt_ < STEADY_CNT_MAX) {
                staticSteadyStateCnt_++;
        } else {
        gyro_cali_.gx_bias += BIAS_ALPHA * (corrDat_.gx);
        gyro_cali_.gx_bias = CLAMP(gyro_cali_.gx_bias, GYRO_BIAS_MAX_RAW);

        gyro_cali_.gy_bias += BIAS_ALPHA * (corrDat_.gy);
        gyro_cali_.gy_bias = CLAMP(gyro_cali_.gy_bias, GYRO_BIAS_MAX_RAW);

        gyro_cali_.gz_bias += BIAS_ALPHA * (corrDat_.gz);
        gyro_cali_.gz_bias = CLAMP(gyro_cali_.gz_bias, GYRO_BIAS_MAX_RAW);
        }
    } else
        staticSteadyStateCnt_ = 0;
    
    return corrDat_;
}

float ImuCalibration::invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i;
    memcpy(&i, &y, sizeof(float));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(float));
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

