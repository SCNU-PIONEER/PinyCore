#include "MahonyAHRS.hpp"
#include "dsp/fast_math_functions.h"

using namespace IMU_MAHONY_AHRS;

#define twoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain

void Mahony::regiter(float _sampleFrequency, float _twoKp = (2.0f * 0.5f),
                     float _twoKi = (2.0f * 0.0f))
{
    twoKp = _twoKp;
    twoKi = _twoKi;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    anglesComputed = 0;
    invSampleFreq = 1.0f / _sampleFrequency;
}

float Mahony::invSqrt(float _x)
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

void Mahony::init(float _ax, float _ay, float _az)
{
    float recipNorm;
    float initYaw, initPitch, initRoll;
    float cr2, cp2, cy2, sr2, sp2, sy2;
    float sinRoll, cosRoll, sinPitch, cosPitch;
    float magX, magY;

    recipNorm = invSqrt((_ax * _ax) + (_ay * _ay) + (_az * _az));
    _ax *= recipNorm;
    _ay *= recipNorm;
    _az *= recipNorm;

    arm_atan2_f32(-_ax, _az, &initPitch);
    arm_atan2_f32(_ay, _az, &initRoll);

    sinRoll = sinf(initRoll);
    cosRoll = cosf(initRoll);
    cosPitch = cosf(initPitch);
    sinPitch = sinf(initPitch);

    initYaw = 0.0f;

    cr2 = cosf(initRoll * 0.5f);
    cp2 = cosf(initPitch * 0.5f);
    cy2 = cosf(initYaw * 0.5f);
    sr2 = sinf(initRoll * 0.5f);
    sp2 = sinf(initPitch * 0.5f);
    sy2 = sinf(initYaw * 0.5f);

    q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    // Normalise quaternion
    recipNorm = invSqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony::update(float _gx, float _gy, float _gz, float _ax, float _ay,
                    float _az, float _dt)
{
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    updateIMU(_gx, _gy, _gz, _ax, _ay, _az, _dt);

    computeAngles();
}
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void Mahony::updateIMU(float _gx, float _gy, float _gz, float _ax, float _ay,
                       float _az, float _dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if ((_ax != 0.0f) || (_ay != 0.0f) || (_az != 0.0f)) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt((_ax * _ax) + (_ay * _ay) + (_az * _az));
        _ax *= recipNorm;
        _ay *= recipNorm;
        _az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (_ay * halfvz - _az * halfvy);
        halfey = (_az * halfvx - _ax * halfvz);
        halfez = (_ax * halfvy - _ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex *
                           invSampleFreq; // integral error scaled by Ki
            integralFBy += twoKi * halfey * invSampleFreq;
            integralFBz += twoKi * halfez * invSampleFreq;
            _gx += integralFBx; // apply integral feedback
            _gy += integralFBy;
            _gz += integralFBz;
        } else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        _gx += twoKpDef * halfex;
        _gy += twoKpDef * halfey;
        _gz += twoKpDef * halfez;
    }

    // Integrate rate of change of quaternion
    _gx *= (0.5f * invSampleFreq); // pre-multiply common factors
    _gy *= (0.5f * invSampleFreq);
    _gz *= (0.5f * invSampleFreq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * _gx - qc * _gy - q3 * _gz);
    q1 += (qa * _gx + qc * _gz - q3 * _gy);
    q2 += (qa * _gy - qb * _gz + q3 * _gx);
    q3 += (qa * _gz + qb * _gy - qc * _gx);

    // Normalise quaternion
    recipNorm = invSqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony::computeAngles()
{
    arm_atan2_f32((q0 * q1) + (q2 * q3), 0.5f - (q1 * q1) - (q2 * q2),
                  &edata_.roll);
    edata_.roll *= 57.29578f;
    edata_.pitch = 57.29578f * asinf(-2.0f * (q1 * q3 - q0 * q2));
    arm_atan2_f32((q1 * q2) + (q0 * q3), 0.5f - (q2 * q2) - (q3 * q3),
                  &edata_.yaw);
    edata_.yaw *= 57.29578f;
    anglesComputed = 1;
}

EData_s Mahony::getEdata() { return edata_; }

float Mahony::getRoll() { return edata_.roll; }

float Mahony::getPitch() { return edata_.pitch; }

float Mahony::getYaw() { return edata_.yaw; }
