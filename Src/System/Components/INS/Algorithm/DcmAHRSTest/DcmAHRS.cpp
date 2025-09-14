#include "DcmAHRS.hpp"
#include "dsp/fast_math_functions.h"

#define GYRO_PREDICTION 1

using namespace IMU_DCM_AHRS_TEST;

void DcmAhrs::init()
{
    // Initialize the X vector with the initial state
    for (uint8_t i = 0; i < STATE_DIM; i++) {
        X_[0][i] = DEFAULT_IMU_ACCEL_GYRO_STATE[i];
    }

    // Initialize the P
    P_.diag() << DEFAULT_Q_DCM2_INIT << DEFAULT_Q_DCM2_INIT
              << DEFAULT_Q_DCM2_INIT << DEFAULT_Q_BIAS2_INIT
              << DEFAULT_Q_BIAS2_INIT << DEFAULT_Q_BIAS2_INIT;
}

void DcmAhrs::updateDCM(float _gx, float _gy, float _gz, float _ax, float _ay,
                        float _az, float _dt)
{
    dt_ = _dt;
    float invG0 = (1.f / DEFAULT_GRAVITY);
    float invG02 = (invG0 * invG0);

    // save last state to memory for rotation estimation
    XL_ = X_;

    // control input (gyroscopes)

    U_[0][0] = _gx;
    U_[1][0] = _gy;
    U_[2][0] = -_gz;

    // state prediction : no linearization f(X, U)
    Matrix<STATE_DIM, 1> xPred;
    // attitude section
    xPred[0][0] =
            X_[0][0] - (dt_ * (U_[1][0] * X_[2][0] - U_[2][0] * X_[1][0] +
                               X_[1][0] * X_[5][0] - X_[2][0] * X_[4][0]));
    xPred[1][0] =
            X_[1][0] + (dt_ * (U_[0][0] * X_[2][0] - U_[2][0] * X_[0][0] +
                               X_[0][0] * X_[5][0] - X_[2][0] * X_[3][0]));
    xPred[2][0] =
            X_[2][0] - (dt_ * (U_[0][0] * X_[1][0] - U_[1][0] * X_[0][0] +
                               X_[0][0] * X_[4][0] - X_[1][0] * X_[3][0]));

    // bias section (Assuming slow change is 0)
#if GYRO_PREDICTION
    xPred[3][0] = 0.f;
    xPred[4][0] = 0.f;
    xPred[5][0] = 0.f;
#else
    xPred[3][0] = X_[3][0];
    xPred[4][0] = X_[4][0];
    xPred[5][0] = X_[5][0];
#endif


    // update F
    F_[0][1] = dt_ * (U_[2][0] - X_[5][0]);  // df0/dx1 = dt*(u2 - x5)
    F_[0][2] = -dt_ * (U_[1][0] - X_[4][0]); // df0/dx2 = -dt*(u1 - x4)
    F_[0][4] = dt_ * X_[2][0];               // df0/dx4 = dt*x2
    F_[0][5] = -dt_ * X_[1][0];              // df0/dx5 = -dt*x1
    F_[1][0] = -dt_ * (U_[2][0] - X_[5][0]); // df1/dx0 = -dt*(u2 - x5)
    F_[1][2] = dt_ * (U_[0][0] - X_[3][0]);  // df1/dx2 = dt*(u0 - x3)
    F_[1][3] = -dt_ * X_[2][0];              // df1/dx3 = -dt*x2
    F_[1][5] = dt_ * X_[0][0];               // df1/dx5 = dt*x0
    F_[2][0] = dt_ * (U_[1][0] - X_[4][0]);  // df2/dx0 = dt*(u1 - x4)
    F_[2][1] = -dt_ * (U_[0][0] - X_[3][0]); // df2/dx1 = -dt*(u0 - x3)
    F_[2][3] = dt_ * X_[1][0];               // df2/dx3 = dt*x1
    F_[2][4] = -dt_ * X_[0][0];              // df2/dx4 = -dt*x0

    // update G
    G_[0][1] = dt_ * X_[2][0];
    G_[0][2] = -dt_ * X_[1][0];
    G_[1][0] = -dt_ * X_[2][0];
    G_[1][2] = dt_ * X_[0][0];
    G_[2][0] = dt_ * X_[1][0];
    G_[2][1] = -dt_ * X_[0][0];
    // the remaining is eye(3)

    // update Q
    float dt2 = dt_ * dt_;
    Q_[0][0] = dt2 * DCMVariance_;
    Q_[1][1] = dt2 * DCMVariance_;
    Q_[2][2] = dt2 * DCMVariance_;
    Q_[3][3] = dt2 * biasVariance_;
    Q_[4][4] = dt2 * biasVariance_;
    Q_[5][5] = dt2 * biasVariance_;

    // P = F * P * F' + G * Q * G'
    P_ = F_ * P_ * F_.trans() + G_ * Q_ * G_.trans();

    // measurement (accelerometer)
    Z_[0][0] = _ax * invG0;
    Z_[1][0] = _ay * invG0;
    Z_[2][0] = _az * invG0;

    // Kalman innovation
    Y_[0][0] = Z_[0][0] - X_[0][0];
    Y_[1][0] = Z_[1][0] - X_[1][0];
    Y_[2][0] = Z_[2][0] - X_[2][0];

    // build R
    float len = 0;
    arm_sqrt_f32((Y_.trans() * Y_)[0][0], &len);
    float aLen = len * DEFAULT_GRAVITY;
    float rAdab =
            (measurementVariance_ + aLen * measurementVarianceVariableGain_) *
            invG02;
    R_ = Matrix<MEAS_DIM, MEAS_DIM>::diag(rAdab);

    // S = H * P * H^T + R
    S_ = H_ * P_ * H_.trans() + R_;

    // verify that the innovation covariance is large enough
    std::fmax(S_[0][0], VARIANCE_MIN_LIMIT);
    std::fmax(S_[1][1], VARIANCE_MIN_LIMIT);
    std::fmax(S_[2][2], VARIANCE_MIN_LIMIT);

    // determinant of S
    float detS = (-S_[0][0] * (S_[1][2] * S_[1][2])) -
                 ((S_[0][2] * S_[0][2]) * S_[1][1]) -
                 ((S_[0][1] * S_[0][1]) * S_[2][2]) +
                 (S_[0][1] * S_[0][2] * S_[1][2] * 2.0f) +
                 (S_[0][0] * S_[1][1] * S_[2][2]);
    if (detS == 0.0f)
        return;

    // Kalman gain: K = P * H^T * S^-1
    K_ = P_ * H_.trans() * S_.inv();

    // X = xPred + K * Y_
    X_ = xPred + K_ * Y_;

#if GYRO_PREDICTION
    X_[3][0] = 0.0f;
    X_[4][0] = 0.0f;
    X_[5][0] = 0.0f;
#else
    X_[3][0] = std::fmax(-GYRO_BIAS_MAX, std::fmin(X_[3][0], GYRO_BIAS_MAX));
    X_[4][0] = std::fmax(-GYRO_BIAS_MAX, std::fmin(X_[4][0], GYRO_BIAS_MAX));
    X_[5][0] = std::fmax(-GYRO_BIAS_MAX, std::fmin(X_[5][0], GYRO_BIAS_MAX));
#endif

    // Update covariance: P = (I - K * H) * P
    P_ = (Matrix<STATE_DIM, STATE_DIM>::eye() - K_ * H_) * P_;

    // Normalization of covariance
    float invLen = invSqrt((X_.trans() * X_)[0][0]);

    // increment covariance slightly at each iteration (nonoptimal but keeps the
    // filter stable against rounding errors in 32bit float computation)
    P_[0][0] += VARIANCE_SAFETY_INCREMENT;
    P_[1][1] += VARIANCE_SAFETY_INCREMENT;
    P_[2][2] += VARIANCE_SAFETY_INCREMENT;
    //	P_[3][3] += VARIANCE_SAFETY_INCREMENT;
    //	P_[4][4] += VARIANCE_SAFETY_INCREMENT;
    //	P_[5][5] += VARIANCE_SAFETY_INCREMENT;

    // variance is required to be always at least the minimum value
    std::fmax(P_[0][0], VARIANCE_MIN_LIMIT);
    std::fmax(P_[1][1], VARIANCE_MIN_LIMIT);
    std::fmax(P_[2][2], VARIANCE_MIN_LIMIT);
    std::fmax(P_[3][3], VARIANCE_MIN_LIMIT);
    std::fmax(P_[4][4], VARIANCE_MIN_LIMIT);
    std::fmax(P_[5][5], VARIANCE_MIN_LIMIT);

    // normalized a posteriori state
    X_ *= invLen;
    Matrix<MEAS_DIM, 1> uNb = U_ - X_.block<MEAS_DIM, 1>(0, 0);

    // calculate the second row (sr) from a rotated first row (rotation with bias
    // corrected gyroscope measurement)
    // sr = fr × XL - dt * XL × (fr × uNb)
    Matrix<3, 1> xLBias = XL_.block<MEAS_DIM, 1>(0, 0);
    SR_ = Matrix<3, 1>::cross(FR_, xLBias) -
          dt_ * Matrix<3, 1>::cross(xLBias, Matrix<3, 1>::cross(FR_, uNb));
}

void DcmAhrs::computeAngles()
{
    // alternative method estimating the whole rotation matrix
    // integrate full rotation matrix (using first row estimate in memory)
    // normalize the second row
    float invlen = invSqrt((SR_.trans() * SR_)[0][0]);
    SR_[0][0] *= invlen;
    SR_[1][0] *= invlen;
    SR_[2][0] *= invlen;

    // recompute the first row (ensure perpendicularity)
    FR_ = Matrix<3, 1>::cross(SR_, XL_.block<MEAS_DIM, 1>(0, 0));

    // normalize the first row
    invlen = invSqrt((FR_.trans() * FR_)[0][0]);
    FR_[0][0] *= invlen;
    FR_[1][0] *= invlen;
    FR_[2][0] *= invlen;

    arm_atan2_f32(SR_[0][0], FR_[0][0], &edata_.yaw);
    arm_atan2_f32(X_[1][0], X_[2][0], &edata_.roll);
    edata_.pitch = asinf(X_[0][0]);
}

void DcmAhrs::update(float _gx, float _gy, float _gz, float _ax, float _ay,
                     float _az, float _dt)
{
    updateDCM(_gx, _gy, _gz, _ax, _ay, _az, _dt);

    // compute the angles from the current state
    computeAngles();
}

EData_s DcmAhrs::getEdata() { return edata_; }

float DcmAhrs::getRoll() { return edata_.roll; }

float DcmAhrs::getPitch() { return edata_.pitch; }

float DcmAhrs::getYaw() { return edata_.yaw; }

void DcmAhrs::getQuaternion(float *_q)
{
    _q[0] = X_[0][0];
    _q[1] = X_[1][0];
    _q[2] = X_[2][0];
    _q[3] = X_[3][0];
}

float DcmAhrs::invSqrt(float _x)
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
