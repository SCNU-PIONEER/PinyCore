#include "MahonyAHRS.hpp"
#include "dsp/fast_math_functions.h"

namespace IMU_MahonyAHRS {

#define twoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain

void Mahony::regiter(float sampleFrequency, float _twoKp = (2.0f * 0.5f),
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
    invSampleFreq = 1.0f / sampleFrequency;
}

float Mahony::invSqrt(float x)
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

void Mahony::init(float ax, float ay, float az)
{
    float recipNorm;
    float init_yaw, init_pitch, init_roll;
    float cr2, cp2, cy2, sr2, sp2, sy2;
    float sin_roll, cos_roll, sin_pitch, cos_pitch;
    float magX, magY;

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    init_pitch = atan2f(-ax, az);
    init_roll = atan2f(ay, az);

    sin_roll = sinf(init_roll);
    cos_roll = cosf(init_roll);
    cos_pitch = cosf(init_pitch);
    sin_pitch = sinf(init_pitch);

    init_yaw = 0.0f;

    cr2 = cosf(init_roll * 0.5f);
    cp2 = cosf(init_pitch * 0.5f);
    cy2 = cosf(init_yaw * 0.5f);
    sr2 = sinf(init_roll * 0.5f);
    sp2 = sinf(init_pitch * 0.5f);
    sy2 = sinf(init_yaw * 0.5f);

    q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony::update(fdata_s *_imu_data)
{
    float gx = _imu_data->gx;
    float gy = _imu_data->gy;
    float gz = _imu_data->gz;
    float ax = _imu_data->ax;
    float ay = _imu_data->ay;
    float az = _imu_data->az;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    updateIMU(gx, gy, gz, ax, ay, az);

    computeAngles();
}
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void Mahony::updateIMU(float gx, float gy, float gz, float ax, float ay,
                       float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex *
                           invSampleFreq; // integral error scaled by Ki
            integralFBy += twoKi * halfey * invSampleFreq;
            integralFBz += twoKi * halfez * invSampleFreq;
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * invSampleFreq); // pre-multiply common factors
    gy *= (0.5f * invSampleFreq);
    gz *= (0.5f * invSampleFreq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony::computeAngles()
{
    arm_atan2_f32(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2, &edata_.roll);
    edata_.roll *= 57.29578f;
    edata_.pitch = 57.29578f * asinf(-2.0f * (q1 * q3 - q0 * q2));
    arm_atan2_f32(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3, &edata_.yaw);
    edata_.yaw *= 57.29578f;
    anglesComputed = 1;
}

edata_s Mahony::getEdata(void) { return edata_; }

float Mahony::getRoll(void) { return edata_.roll; }

float Mahony::getPitch(void) { return edata_.pitch; }

float Mahony::getYaw(void) { return edata_.yaw; }


}
