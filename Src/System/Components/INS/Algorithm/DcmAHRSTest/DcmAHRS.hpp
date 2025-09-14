#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>

#include "Matrix.hpp"

namespace IMU_DCM_AHRS_TEST {

struct EData_s {
    float roll;  // Roll angle in radians
    float pitch; // Pitch angle in radians
    float yaw;   // Yaw angle in radians
};

class DcmAhrs {
    static constexpr int STATE_DIM = 6;
    static constexpr int MEAS_DIM = 3;
    static constexpr int CONTROL_DIM = 3;

    static constexpr float DEFAULT_IMU_ACCEL_GYRO_STATE[6] = {
        0, 0, 1, 0, 0, 0
    };
    static constexpr float DEFAULT_GRAVITY =
            (9.7833f); // Standard gravity in m/s^2

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

    static constexpr float GYRO_BIAS_MAX = (1.0f * 0.01745f);


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

    Matrix<STATE_DIM, 1> X_ = Matrix<STATE_DIM, 1>::zeros();

    Matrix<STATE_DIM, 1> XL_ = Matrix<STATE_DIM, 1>::zeros(); // x last

    Matrix<MEAS_DIM, 1> Z_ = Matrix<MEAS_DIM, 1>::zeros();

    Matrix<CONTROL_DIM, 1> U_ = Matrix<CONTROL_DIM, 1>::zeros();

    Matrix<STATE_DIM, STATE_DIM> F_ = Matrix<STATE_DIM, STATE_DIM>::zeros();

    Matrix<STATE_DIM, STATE_DIM> G_ =
            Matrix<STATE_DIM,
                   STATE_DIM>::eye(); // Jacobian matrix of process noise

    Matrix<STATE_DIM, STATE_DIM> P_ = Matrix<STATE_DIM, STATE_DIM>::eye();

    Matrix<STATE_DIM, MEAS_DIM> K_ = Matrix<STATE_DIM, MEAS_DIM>::zeros();

    Matrix<MEAS_DIM, 1> Y_ = Matrix<MEAS_DIM, 1>::zeros(); // Kalman innovation

    Matrix<STATE_DIM, STATE_DIM> Q_ = Matrix<STATE_DIM, STATE_DIM>::eye();

    Matrix<MEAS_DIM, STATE_DIM> H_ = { { 1.f, 0, 0, 0, 0, 0 },
                                       { 0, 1.f, 0, 0, 0, 0 },
                                       { 0, 0, 1.f, 0, 0, 0 } };

    Matrix<MEAS_DIM, MEAS_DIM> R_ = Matrix<MEAS_DIM, MEAS_DIM>::eye();

    Matrix<MEAS_DIM, MEAS_DIM> S_ =
            Matrix<MEAS_DIM, MEAS_DIM>::zeros(); // Innovation Covariance

    // first row for alternative rotation computation
    // default is yaw = 0 which happens when fr = [1, 0, 0]
    Matrix<3, 1> FR_{ { 1 }, { 0 }, { 0 } }; // First row^T of rotation matrix
    // second row for alternative rotation computation
    // default is pitch = 0 which happens when fr = [0, 1, 0]
    Matrix<3, 1> SR_{ { 0 }, { 1 }, { 0 } }; // Second row^T of rotation matrix
};

} // namespace IMU_DCM_AHRS_TEST
