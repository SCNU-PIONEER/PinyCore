#include <cmath>
#include <cstdint>
#include "calibrate.hpp"

#include "dsp/fast_math_functions.h"

#define IMU_RAW_RANGE (32768)

using namespace INS_SYS;

void IMUCalibration::init(const AccCali_s &_accCali,
                          const GyroCali_s &_gyroCali, float _aTransK,
                          float _gTransK)
{
    acc_cali_ = _accCali;
    gyro_cali_ = _gyroCali;
    staticSteadyStateCnt_ = 0;
    aTransK_ = _aTransK;
    gTransK_ = _gTransK;
}

void IMUCalibration::correctA(int16_t _ax, int16_t _ay, int16_t _az)
{
    if constexpr (CORRECT_IMU_DATA) {
        float axUb = static_cast<float>(_ax) - acc_cali_.accel_offs[0];
        float ayUb = static_cast<float>(_ay) - acc_cali_.accel_offs[1];
        float azUb = static_cast<float>(_az) - acc_cali_.accel_offs[2];
        corrDat_.ax = (acc_cali_.accel_T[0][0] * axUb +
                       acc_cali_.accel_T[0][1] * ayUb +
                       acc_cali_.accel_T[0][2] * azUb) *
                      aTransK_;
        corrDat_.ay = (acc_cali_.accel_T[1][0] * axUb +
                       acc_cali_.accel_T[1][1] * ayUb +
                       acc_cali_.accel_T[1][2] * azUb) *
                      aTransK_;
        corrDat_.az = (acc_cali_.accel_T[2][0] * axUb +
                       acc_cali_.accel_T[2][1] * ayUb +
                       acc_cali_.accel_T[2][2] * azUb) *
                      aTransK_;
    } else {
        corrDat_.ax = static_cast<float>(_ax) * aTransK_;
        corrDat_.ay = static_cast<float>(_ay) * aTransK_;
        corrDat_.az = static_cast<float>(_az) * aTransK_;
    }
}

void IMUCalibration::correctG(int16_t _gx, int16_t _gy, int16_t _gz)
{
    if constexpr (CORRECT_IMU_DATA) {
        corrDat_.gx =
                ((static_cast<float>(_gx)) - gyro_cali_.gx_bias -
                 (gyro_cali_.gx_tco_k * temperature_ + gyro_cali_.gx_tco_b0)) *
                gTransK_;
        corrDat_.gy =
                ((static_cast<float>(_gy)) - gyro_cali_.gy_bias -
                 (gyro_cali_.gy_tco_k * temperature_ + gyro_cali_.gy_tco_b0)) *
                gTransK_;
        corrDat_.gz =
                ((static_cast<float>(_gz)) - gyro_cali_.gz_bias -
                 (gyro_cali_.gz_tco_k * temperature_ + gyro_cali_.gz_tco_b0)) *
                gTransK_;
    } else {
        corrDat_.gx = static_cast<float>(_gx) * gTransK_;
        corrDat_.gy = static_cast<float>(_gy) * gTransK_;
        corrDat_.gz = static_cast<float>(_gz) * gTransK_;
    }
}

void IMUCalibration::correctM(int16_t _mx, int16_t _my, int16_t _mz)
{
    // TODO: magnetometer calibration
}

CaliOutput_s IMUCalibration::steadyStateDetection()
{
    float recipNorm = 0;
    arm_sqrt_f32(((corrDat_.ax * corrDat_.ax) + (corrDat_.ay * corrDat_.ay) +
                  (corrDat_.az * corrDat_.az)),
                 &recipNorm);
    if (((G * recipNorm) < (1 + STEADY_ACCEL_RANGE)) &&
        ((G * recipNorm) > (1 - STEADY_ACCEL_RANGE)) &&
        fabsf(corrDat_.gx) < STEADY_GYRO_RANGE &&
        fabsf(corrDat_.gy) < STEADY_GYRO_RANGE &&
        fabsf(corrDat_.gz) < STEADY_GYRO_RANGE) {
        if (staticSteadyStateCnt_ < STEADY_CNT_MAX) {
            staticSteadyStateCnt_++;
        } else {
            gyro_cali_.gx_bias += BIAS_ALPHA * (corrDat_.gx);
            gyro_cali_.gx_bias =
                    std::fmax(-GYRO_BIAS_MAX_RAW,
                              std::fmin(gyro_cali_.gx_bias, GYRO_BIAS_MAX_RAW));

            gyro_cali_.gy_bias += BIAS_ALPHA * (corrDat_.gy);
            gyro_cali_.gy_bias =
                    std::fmax(-GYRO_BIAS_MAX_RAW,
                              std::fmin(gyro_cali_.gy_bias, GYRO_BIAS_MAX_RAW));

            gyro_cali_.gz_bias += BIAS_ALPHA * (corrDat_.gz);
            gyro_cali_.gz_bias =
                    std::fmax(-GYRO_BIAS_MAX_RAW,
                              std::fmin(gyro_cali_.gz_bias, GYRO_BIAS_MAX_RAW));
        }
    } else
        staticSteadyStateCnt_ = 0;

    return corrDat_;
}
