#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
namespace IMU_DCM_AHRS {

struct EData_s {
    float roll;  // Roll angle in radians
    float pitch; // Pitch angle in radians
    float yaw;   // Yaw angle in radians
};

class DcmAhrs {
    static constexpr float DEFAULT_IMU_ACCEL_GYRO_STATE[6] = {
        0, 0, 1, 0, 0, 0
    };

    static constexpr float G = (9.7833f); // GuangZhou gravity in m/s^2
    static constexpr float INV_G = (1.f / G);
    static constexpr float INV_G2 = (1.f / (G * G));

    static constexpr float DEFAULT_Q_DCM2_INIT = (1.8f * 1.8f);
    static constexpr float DEFAULT_Q_BIAS2_INIT = (0.1f * 0.1f);

    static constexpr float DEFAULT_R_MEASUREMENT2 = (0.003f * 0.003f);
    static constexpr float DEFAULT_R_MEASUREMENT2_VARIABLE_GAIN = (10.f * 10.f);

    static constexpr float DEFAULT_Q_DCM2 = (0.0037f * 0.0037f);
    static constexpr float DEFAULT_Q_BIAS2 = (2.5e-3f * 2.5e-3f);

    static constexpr float VARIANCE_MIN_LIMIT = (0.0001f * 0.0001f);
    // set this to a small positive number or 0 to disable the feature.
    static constexpr float VARIANCE_SAFETY_INCREMENT = (0.00001f * 0.00001f);
    // set this to a small positive number or 0 to disable the feature.

public:
    DcmAhrs(float _sampleFrequency, float _dcmVariance = DEFAULT_Q_DCM2,
            float _biasVariance = DEFAULT_Q_BIAS2,
            float _measurementVariance = DEFAULT_R_MEASUREMENT2,
            float _measurementVarianceVariableGain =
                    DEFAULT_R_MEASUREMENT2_VARIABLE_GAIN)
            : dt_(_sampleFrequency)
            , DCMVariance_(_dcmVariance)
            , biasVariance_(_biasVariance)
            , measurementVariance_(_measurementVariance)
            , measurementVarianceVariableGain_(_measurementVarianceVariableGain)
    {
    }

    void init();

    void update(float _gx, float _gy, float _gz, float _ax, float _ay,
                float _az, float _dt);

    EData_s getEdata();

    float getRoll();
    float getPitch();
    float getYaw();

    void getQuaternion(float *_q);

protected:
    float invSqrt(float _x);
    void updateDCM(float _gx, float _gy, float _gz, float _ax, float _ay,
                   float _az, float _dt);
    void computeAngles();

private:
    EData_s edata_; // Euler angles data

    uint16_t staticStateCnt_ = 0;

    float dt_; // Sample time

    // DCM parameters
    float DCMVariance_; // a variance for DCM state update, Q(0,0), Q(1,1), and Q(2,2)
    float biasVariance_; // a variance for bias state update, Q(3,3), Q(4,4), and Q(5,5)
    float measurementVariance_; // variance of calibrated accelerometer (g-component)
    float measurementVarianceVariableGain_; // large variance for some unknown acceleration (acc = a + g)

    // X state vector, P covariance matrix, and DCM matrix
    float x_last_[3];
    float x0_, x1_, x2_, x3_, x4_, x5_;
    float fr0_, fr1_, fr2_, sr0_, sr1_, sr2_;

    float P00_, P01_, P02_, P03_, P04_, P05_;
    float P11_, P12_, P13_, P14_, P15_;
    float P22_, P23_, P24_, P25_;
    float P33_, P34_, P35_;
    float P44_, P45_;
    float P55_;
};

} // namespace IMU_DCM_AHRS
