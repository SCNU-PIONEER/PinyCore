/*
 * ===== Model =====
 * accel_corr = accel_T * (accel_raw - accel_offs)
 *
 * accel_corr[3] - fully corrected acceleration vector in body frame
 * accel_T[3][3] - accelerometers transform matrix, rotation and scaling
 * transform accel_raw[3]  - raw acceleration vector accel_offs[3] -
 * acceleration offset vector
 *
 * ===== Calibration =====
 *
 * Reference vectors
 * accel_corr_ref[6][3] = [  g  0  0 ]     // nose up			ACC_PX
 *                        | -g  0  0 |     // nose down			ACC_NX
 *                        |  0  g  0 |     // left side down	ACC_PY
 *                        |  0 -g  0 |     // right side down	ACC_NY
 *                        |  0  0  g |     // on back			ACC_PZ
 *                        [  0  0 -g ]     // level
 * ACC_NZ accel_raw_ref[6][3]
 *
 * accel_corr_ref[i] = accel_T * (accel_raw_ref[i] - accel_offs), i = 0...5
 *
 * 6 reference vectors * 3 axes = 18 equations
 * 9 (accel_T) + 3 (accel_offs) = 12 unknown constants
 *
 * Find accel_offs
 *
 * accel_offs[i] = (accel_raw_ref[i*2][i] + accel_raw_ref[i*2+1][i]) / 2
 *
 * Find accel_T
 *
 * 9 unknown constants
 * need 9 equations -> use 3 of 6 measurements -> 3 * 3 = 9 equations
 *
 * accel_corr_ref[i*2] = accel_T * (accel_raw_ref[i*2] - accel_offs), i = 0...2
 *
 * Solve separate system for each row of accel_T:
 *
 * accel_corr_ref[j*2][i] = accel_T[i] * (accel_raw_ref[j*2] - accel_offs), j =
 * 0...2
 *
 * A * x = b
 *
 * x = [ accel_T[0][i] ]
 *     | accel_T[1][i] |
 *     [ accel_T[2][i] ]
 *
 * b = [ accel_corr_ref[0][i] ]	// One measurement per side is enough
 *     | accel_corr_ref[2][i] |
 *     [ accel_corr_ref[4][i] ]
 *
 * a[i][j] = accel_raw_ref[i][j] - accel_offs[j], i = 0;2;4, j = 0...2
 *
 * Matrix A is common for all three systems:
 * A = [ a[0][0]  a[0][1]  a[0][2] ]
 *     | a[2][0]  a[2][1]  a[2][2] |
 *     [ a[4][0]  a[4][1]  a[4][2] ]
 *
 * x = A^-1 * b
 *
 * accel_T = A^-1 * g
 */
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

void IMUCalibration::steadyStateDetection()
{
    float recipNorm = 0;
    arm_sqrt_f32(((corrDat_.ax * corrDat_.ax) + (corrDat_.ay * corrDat_.ay) +
                  (corrDat_.az * corrDat_.az)),
                 &recipNorm);
    if (((G / recipNorm) < (1 + STEADY_ACCEL_RANGE)) &&
        ((G / recipNorm) > (1 - STEADY_ACCEL_RANGE)) &&
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
}
