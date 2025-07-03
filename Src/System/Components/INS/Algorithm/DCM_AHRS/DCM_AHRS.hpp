#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>

#define DEFAULT_GRAVITY (9.80665f) // Standard gravity in m/s^2

static const float DEFAULT_IMU_ACCEL_GYRO_STATE[6] = {0, 0, 1, 0, 0, 0};

#define DEFAULT_Q_DCM2_INIT (1.8f * 1.8f)
#define DEFAULT_Q_BIAS2_INIT (0.1f * 0.1f)

#define DEFAULT_R_MEASUREMENT2 (0.003f * 0.003f)
#define DEFAULT_R_MEASUREMENT2_VARIABLE_GAIN (10.f * 10.f)

#define DEFAULT_Q_DCM2                       (0.0037f * 0.0037f)
#define DEFAULT_Q_BIAS2                      (2.5e-3f * 2.5e-3f)


#define VARIANCE_MIN_LIMIT                   (0.0001f * 0.0001f)
// set this to a small positive number or 0 to disable the feature.
#define VARIANCE_SAFETY_INCREMENT            (0.00001f * 0.00001f)
// set this to a small positive number or 0 to disable the feature.


namespace IMU_DCM_AHRS {

struct edata_s {
    float roll;  // Roll angle in radians
    float pitch; // Pitch angle in radians
    float yaw;   // Yaw angle in radians
};

class DCM_AHRS {
public:
    DCM_AHRS(float _sampleFrequency,
             float _DCMVarianceInit = DEFAULT_Q_DCM2_INIT,
             float _DCMVariance = DEFAULT_Q_DCM2,
             float _biasVarianceInit = DEFAULT_Q_BIAS2_INIT,
             float _biasVariance = DEFAULT_Q_BIAS2,
             float _measurementVariance = DEFAULT_R_MEASUREMENT2,
             float _measurementVarianceVariableGain =
                     DEFAULT_R_MEASUREMENT2_VARIABLE_GAIN)
            : dt_(_sampleFrequency)
            , DCMVarianceInit_(_DCMVarianceInit)
            , biasVarianceInit_(_biasVarianceInit)
            , DCMVariance_(_DCMVariance)
            , biasVariance_(_biasVariance)
            , measurementVariance_(_measurementVariance)
            , measurementVarianceVariableGain_(_measurementVarianceVariableGain)
    {
    }

    void init();

    void update(float _gx, float _gy, float _gz, float _ax, float _ay,
                float _az, float _dt);

    edata_s getEdata(void);

    float getRoll(void);
    float getPitch(void);
    float getYaw(void);

    void getQuaternion(float *q);

protected:
    float invSqrt(float x);
    void updateDCM(float _gx, float _gy, float _gz, float _ax, float _ay,
                   float _az, float _dt);
    void computeAngles();

private:
    template <typename T>
    T CLAMP(T value, T max) {
        return std::max(-max, std::min(value, max));
    }

    edata_s edata_; // Euler angles data
            
    uint16_t staticStateCnt_ = 0;

    float g_ = DEFAULT_GRAVITY;

    float linear_a_[3];

    float dt_; // Sample time

    const float *initState_ = DEFAULT_IMU_ACCEL_GYRO_STATE;

    // DCM parameters
    float DCMVarianceInit_;
    float biasVarianceInit_;
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

}
