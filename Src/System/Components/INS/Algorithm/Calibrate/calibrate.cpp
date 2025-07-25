#include <math.h>
#include <stdlib.h>
#include "calibrate.hpp"

#define IMU_RAW_RANGE (32768)

void IMUCalibration::init(const AccCali_s &_accCali,
                          const GyroCali_s &_gyroCali, const float _temp)
{
    acc_cali_ = _accCali;
    gyro_cali_ = _gyroCali;
    staticSteadyStateCnt_ = 0;
}
imu_data_fp_t IMUCalibration::correct(float _aTransK, float _gTransK, float _gx,
                                      float _gy, float _gz, float _ax,
                                      float _ay, float _az, float _temperature)
{
    // de_ai or de_gi mean the raw decoded data to affinite Calibrate Algorithm
    // see the next function 'CorrectInt16' to understand this abstraction
    float deAx = _ax / _aTransK, deAy = _ay / _aTransK, deAz = _az / _aTransK;
    float deGx = _gx / _gTransK, deGy = _gy / _gTransK, deGz = _gz / _gTransK;
    float axUb = deAx - acc_cali_.accel_offs[0];
    float ayUb = deAy - acc_cali_.accel_offs[1];
    float azUb = deAz - acc_cali_.accel_offs[2];
    corrDat_.ax =
            (acc_cali_.accel_T[0][0] * axUb + acc_cali_.accel_T[0][1] * ayUb +
             acc_cali_.accel_T[0][2] * azUb) *
            _aTransK;
    corrDat_.ay =
            (acc_cali_.accel_T[1][0] * axUb + acc_cali_.accel_T[1][1] * ayUb +
             acc_cali_.accel_T[1][2] * azUb) *
            _aTransK;
    corrDat_.az =
            (acc_cali_.accel_T[2][0] * axUb + acc_cali_.accel_T[2][1] * ayUb +
             acc_cali_.accel_T[2][2] * azUb) *
            _aTransK;

    corrDat_.gx =
            (deGx - gyro_cali_.gx_bias -
             (gyro_cali_.gx_tco_k * _temperature + gyro_cali_.gx_tco_b0)) *
            _gTransK;
    corrDat_.gy =
            (deGy - gyro_cali_.gy_bias -
             (gyro_cali_.gy_tco_k * _temperature + gyro_cali_.gy_tco_b0)) *
            _gTransK;
    corrDat_.gz =
            (deGz - gyro_cali_.gz_bias -
             (gyro_cali_.gz_tco_k * _temperature + gyro_cali_.gz_tco_b0)) *
            _gTransK;

    return corrDat_;
}
imu_data_fp_t IMUCalibration::correctInt16(float _aTransK, float _gTransK,
                                           int16_t _gx, int16_t _gy,
                                           int16_t _gz, int16_t _ax,
                                           int16_t _ay, int16_t _az,
                                           float _temperature)
{
    float axUb = static_cast<float>(_ax) - acc_cali_.accel_offs[0];
    float ayUb = static_cast<float>(_ay) - acc_cali_.accel_offs[1];
    float azUb = static_cast<float>(_az) - acc_cali_.accel_offs[2];
    corrDat_.ax =
            (acc_cali_.accel_T[0][0] * axUb + acc_cali_.accel_T[0][1] * ayUb +
             acc_cali_.accel_T[0][2] * azUb) *
            _aTransK;
    corrDat_.ay =
            (acc_cali_.accel_T[1][0] * axUb + acc_cali_.accel_T[1][1] * ayUb +
             acc_cali_.accel_T[1][2] * azUb) *
            _aTransK;
    corrDat_.az =
            (acc_cali_.accel_T[2][0] * axUb + acc_cali_.accel_T[2][1] * ayUb +
             acc_cali_.accel_T[2][2] * azUb) *
            _aTransK;

    corrDat_.gx =
            ((static_cast<float>(_gx)) - gyro_cali_.gx_bias -
             (gyro_cali_.gx_tco_k * _temperature + gyro_cali_.gx_tco_b0)) *
            _gTransK;
    corrDat_.gy =
            ((static_cast<float>(_gy)) - gyro_cali_.gy_bias -
             (gyro_cali_.gy_tco_k * _temperature + gyro_cali_.gy_tco_b0)) *
            _gTransK;
    corrDat_.gz =
            ((static_cast<float>(_gz)) - gyro_cali_.gz_bias -
             (gyro_cali_.gz_tco_k * _temperature + gyro_cali_.gz_tco_b0)) *
            _gTransK;

    return corrDat_;
}

imu_data_fp_t IMUCalibration::steadyStateDetection()
{
    float recipNorm =
            invSqrt((corrDat_.ax * corrDat_.ax) + (corrDat_.ay * corrDat_.ay) +
                    (corrDat_.az * corrDat_.az));
    if (((g_ * recipNorm) < (1 + STEADY_ACCEL_RANGE)) &&
        ((g_ * recipNorm) > (1 - STEADY_ACCEL_RANGE)) &&
        fabsf(corrDat_.gx) < STEADY_GYRO_RANGE &&
        fabsf(corrDat_.gy) < STEADY_GYRO_RANGE &&
        fabsf(corrDat_.gz) < STEADY_GYRO_RANGE) {
        if (staticSteadyStateCnt_ < STEADY_CNT_MAX) {
            staticSteadyStateCnt_++;
        } else {
            gyro_cali_.gx_bias += BIAS_ALPHA * (corrDat_.gx);
            gyro_cali_.gx_bias = clamp(gyro_cali_.gx_bias, GYRO_BIAS_MAX_RAW);

            gyro_cali_.gy_bias += BIAS_ALPHA * (corrDat_.gy);
            gyro_cali_.gy_bias = clamp(gyro_cali_.gy_bias, GYRO_BIAS_MAX_RAW);

            gyro_cali_.gz_bias += BIAS_ALPHA * (corrDat_.gz);
            gyro_cali_.gz_bias = clamp(gyro_cali_.gz_bias, GYRO_BIAS_MAX_RAW);
        }
    } else
        staticSteadyStateCnt_ = 0;

    return corrDat_;
}

float IMUCalibration::invSqrt(float _x)
{
    float halfx = 0.5f * _x;
    float y = _x;
    long i;
    memcpy(&i, &y, sizeof(float));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(float));
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
