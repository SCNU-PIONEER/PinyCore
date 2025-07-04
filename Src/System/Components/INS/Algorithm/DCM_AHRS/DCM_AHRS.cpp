#include "DCM_AHRS.hpp"
#include "dsp/fast_math_functions.h"

#define AUTO_STATIC_DETECTTION 1

namespace IMU_DCM_AHRS {

void DCM_AHRS::init()
{
    // Initialize the X vector with the initial state
    x0_ = initState_[0];
    x1_ = initState_[1];
    x2_ = initState_[2];
    x3_ = initState_[3];
    x4_ = initState_[4];
    x5_ = initState_[5];

    // Initialize the covariance matrix P
    P00_ = DCMVarianceInit_;
    P01_ = 0;
    P02_ = 0;
    P03_ = 0;
    P04_ = 0;
    P05_ = 0;
    P11_ = DCMVarianceInit_;
    P12_ = 0;
    P13_ = 0;
    P14_ = 0;
    P15_ = 0;
    P22_ = DCMVarianceInit_;
    P23_ = 0;
    P24_ = 0;
    P25_ = 0;
    P33_ = biasVarianceInit_;
    P34_ = 0;
    P35_ = 0;
    P44_ = biasVarianceInit_;
    P45_ = 0;
    P55_ = biasVarianceInit_;

    // first row for alternative rotation computation
    // default is yaw = 0 which happens when fr = [1, 0, 0]
    fr0_ = 1.0f;
    fr1_ = 0.0f;
    fr2_ = 0.0f;
    // second row for alternative rotation computation
    // default is pitch = 0 which happens when fr = [0, 1, 0]
    sr0_ = 0.0f;
    sr1_ = 1.0f;
    sr2_ = 0.0f;
}

void DCM_AHRS::updateDCM(float _gx, float _gy, float _gz, float _ax, float _ay,
                         float _az, float _dt)
{
    dt_ = _dt;
    float INV_G0 = (1.f / dt_);
    float INV_G0_2 = (INV_G0 * INV_G0);
    // save last state to memory for rotation estimation
    x_last_[0] = x0_;
    x_last_[1] = x1_;
    x_last_[2] = x2_;

    // control input (gyroscopes)
    float u0 = 0.0f, u1 = 0.0f, u2 = 0.0f;

    // control input (gyroscopes)
    u0 = _gx;
    u1 = _gy;
    u2 = _gz;

    // state prediction
    float x_0 = x0_ - dt_ * (u1 * x2_ - u2 * x1_ + x1_ * x5_ - x2_ * x4_);
    float x_1 = x1_ + dt_ * (u0 * x2_ - u2 * x0_ + x0_ * x5_ - x2_ * x3_);
    float x_2 = x2_ - dt_ * (u0 * x1_ - u1 * x0_ + x0_ * x4_ - x1_ * x3_);
#if AUTO_STATIC_DETECTTION == 0
    float x_3 = x3_;
    float x_4 = x4_;
    float x_5 = x5_;
#endif

    // covariance prediction
    float dt2 = dt_ * dt_;
    float P_00 = P00_ -
                 dt_ * (P05_ * x1_ * 2.0f - P04_ * x2_ * 2.0f +
                        P02_ * (u1 - x4_) * 2.0f - P01_ * (u2 - x5_) * 2.0f) -
                 dt2 * (-DCMVariance_ +
                        x2_ * (P45_ * x1_ - P44_ * x2_ + P24_ * (u1 - x4_) -
                               P14_ * (u2 - x5_)) +
                        x1_ * (P45_ * x2_ - P55_ * x1_ - P25_ * (u1 - x4_) +
                               P15_ * (u2 - x5_)) +
                        (u2 - x5_) * (P15_ * x1_ - P14_ * x2_ +
                                      P12_ * (u1 - x4_) - P11_ * (u2 - x5_)) -
                        (u1 - x4_) * (P25_ * x1_ - P24_ * x2_ +
                                      P22_ * (u1 - x4_) - P12_ * (u2 - x5_)));
    float P_01 = P01_ +
                 dt_ * (P05_ * x0_ - P03_ * x2_ - P15_ * x1_ + P14_ * x2_ +
                        P02_ * (u0 - x3_) - P12_ * (u1 - x4_) -
                        P00_ * (u2 - x5_) + P11_ * (u2 - x5_)) +
                 dt2 * (x2_ * (P35_ * x1_ - P34_ * x2_ + P23_ * (u1 - x4_) -
                               P13_ * (u2 - x5_)) +
                        x0_ * (P45_ * x2_ - P55_ * x1_ - P25_ * (u1 - x4_) +
                               P15_ * (u2 - x5_)) +
                        (u2 - x5_) * (P05_ * x1_ - P04_ * x2_ +
                                      P02_ * (u1 - x4_) - P01_ * (u2 - x5_)) -
                        (u0 - x3_) * (P25_ * x1_ - P24_ * x2_ +
                                      P22_ * (u1 - x4_) - P12_ * (u2 - x5_)));
    float P_02 = P02_ -
                 dt_ * (P04_ * x0_ - P03_ * x1_ + P25_ * x1_ - P24_ * x2_ +
                        P01_ * (u0 - x3_) - P00_ * (u1 - x4_) +
                        P22_ * (u1 - x4_) - P12_ * (u2 - x5_)) -
                 dt2 * (x1_ * (P35_ * x1_ - P34_ * x2_ + P23_ * (u1 - x4_) -
                               P13_ * (u2 - x5_)) -
                        x0_ * (P45_ * x1_ - P44_ * x2_ + P24_ * (u1 - x4_) -
                               P14_ * (u2 - x5_)) +
                        (u1 - x4_) * (P05_ * x1_ - P04_ * x2_ +
                                      P02_ * (u1 - x4_) - P01_ * (u2 - x5_)) -
                        (u0 - x3_) * (P15_ * x1_ - P14_ * x2_ +
                                      P12_ * (u1 - x4_) - P11_ * (u2 - x5_)));
    float P_03 = P03_ - dt_ * (P35_ * x1_ - P34_ * x2_ + P23_ * (u1 - x4_) -
                               P13_ * (u2 - x5_));
    float P_04 = P04_ - dt_ * (P45_ * x1_ - P44_ * x2_ + P24_ * (u1 - x4_) -
                               P14_ * (u2 - x5_));
    float P_05 = P05_ + dt_ * (P45_ * x2_ - P55_ * x1_ - P25_ * (u1 - x4_) +
                               P15_ * (u2 - x5_));
    float P_11 = P11_ +
                 dt_ * (P15_ * x0_ * 2.0f - P13_ * x2_ * 2.0f +
                        P12_ * (u0 - x3_) * 2.0f - P01_ * (u2 - x5_) * 2.0f) -
                 dt2 * (-DCMVariance_ +
                        x2_ * (P35_ * x0_ - P33_ * x2_ + P23_ * (u0 - x3_) -
                               P03_ * (u2 - x5_)) +
                        x0_ * (P35_ * x2_ - P55_ * x0_ - P25_ * (u0 - x3_) +
                               P05_ * (u2 - x5_)) +
                        (u2 - x5_) * (P05_ * x0_ - P03_ * x2_ +
                                      P02_ * (u0 - x3_) - P00_ * (u2 - x5_)) -
                        (u0 - x3_) * (P25_ * x0_ - P23_ * x2_ +
                                      P22_ * (u0 - x3_) - P02_ * (u2 - x5_)));
    float P_12 = P12_ -
                 dt_ * (P14_ * x0_ - P13_ * x1_ - P25_ * x0_ + P23_ * x2_ +
                        P11_ * (u0 - x3_) - P01_ * (u1 - x4_) -
                        P22_ * (u0 - x3_) + P02_ * (u2 - x5_)) +
                 dt2 * (x1_ * (P35_ * x0_ - P33_ * x2_ + P23_ * (u0 - x3_) -
                               P03_ * (u2 - x5_)) -
                        x0_ * (P45_ * x0_ - P34_ * x2_ + P24_ * (u0 - x3_) -
                               P04_ * (u2 - x5_)) +
                        (u1 - x4_) * (P05_ * x0_ - P03_ * x2_ +
                                      P02_ * (u0 - x3_) - P00_ * (u2 - x5_)) -
                        (u0 - x3_) * (P15_ * x0_ - P13_ * x2_ +
                                      P12_ * (u0 - x3_) - P01_ * (u2 - x5_)));
    float P_13 = P13_ + dt_ * (P35_ * x0_ - P33_ * x2_ + P23_ * (u0 - x3_) -
                               P03_ * (u2 - x5_));
    float P_14 = P14_ + dt_ * (P45_ * x0_ - P34_ * x2_ + P24_ * (u0 - x3_) -
                               P04_ * (u2 - x5_));
    float P_15 = P15_ - dt_ * (P35_ * x2_ - P55_ * x0_ - P25_ * (u0 - x3_) +
                               P05_ * (u2 - x5_));
    float P_22 = P22_ -
                 dt_ * (P24_ * x0_ * 2.0f - P23_ * x1_ * 2.0f +
                        P12_ * (u0 - x3_) * 2.0f - P02_ * (u1 - x4_) * 2.0f) -
                 dt2 * (-DCMVariance_ +
                        x1_ * (P34_ * x0_ - P33_ * x1_ + P13_ * (u0 - x3_) -
                               P03_ * (u1 - x4_)) +
                        x0_ * (P34_ * x1_ - P44_ * x0_ - P14_ * (u0 - x3_) +
                               P04_ * (u1 - x4_)) +
                        (u1 - x4_) * (P04_ * x0_ - P03_ * x1_ +
                                      P01_ * (u0 - x3_) - P00_ * (u1 - x4_)) -
                        (u0 - x3_) * (P14_ * x0_ - P13_ * x1_ +
                                      P11_ * (u0 - x3_) - P01_ * (u1 - x4_)));
    float P_23 = P23_ - dt_ * (P34_ * x0_ - P33_ * x1_ + P13_ * (u0 - x3_) -
                               P03_ * (u1 - x4_));
    float P_24 = P24_ + dt_ * (P34_ * x1_ - P44_ * x0_ - P14_ * (u0 - x3_) +
                               P04_ * (u1 - x4_));
    float P_25 = P25_ + dt_ * (P35_ * x1_ - P45_ * x0_ - P15_ * (u0 - x3_) +
                               P05_ * (u1 - x4_));
    float P_33 = P33_ + dt2 * biasVariance_;
    float P_34 = P34_;
    float P_35 = P35_;
    float P_44 = P44_ + dt2 * biasVariance_;
    float P_45 = P45_;
    float P_55 = P55_ + dt2 * biasVariance_;

    // measurements (accelerometers)
    float z0 = _ax * INV_G0;
    float z1 = _ay * INV_G0;
    float z2 = _az * INV_G0;

    // Kalman innovation
    float y0 = z0 - x_0;
    float y1 = z1 - x_1;
    float y2 = z2 - x_2;

    float a_len = sqrtf(y0 * y0 + y1 * y1 + y2 * y2) * g_;
    float r_adab =
            (measurementVariance_ + a_len * measurementVarianceVariableGain_) *
            INV_G0_2;

    // innovation covariance
    float S00 = P_00 + r_adab;
    float S01 = P_01;
    float S02 = P_02;
    float S11 = P_11 + r_adab;
    float S12 = P_12;
    float S22 = P_22 + r_adab;

    // verify that the innovation covariance is large enough
    if (S00 < VARIANCE_MIN_LIMIT)
        S00 = VARIANCE_MIN_LIMIT;
    if (S11 < VARIANCE_MIN_LIMIT)
        S11 = VARIANCE_MIN_LIMIT;
    if (S22 < VARIANCE_MIN_LIMIT)
        S22 = VARIANCE_MIN_LIMIT;

    // determinant of S
    float det_S = -S00 * (S12 * S12) - (S02 * S02) * S11 - (S01 * S01) * S22 +
                  S01 * S02 * S12 * 2.0f + S00 * S11 * S22;
    if (det_S == 0.0f)
        return;

    // Kalman gain
    float invPart = 1.0f / det_S;
    float K00 = -(S02 * (P_02 * S11 - P_01 * S12) -
                  S01 * (P_02 * S12 - P_01 * S22) + P_00 * (S12 * S12) -
                  P_00 * S11 * S22) *
                invPart;
    float K01 = -(S12 * (P_02 * S00 - P_00 * S02) -
                  S01 * (P_02 * S02 - P_00 * S22) + P_01 * (S02 * S02) -
                  P_01 * S00 * S22) *
                invPart;
    float K02 = -(S12 * (P_01 * S00 - P_00 * S01) -
                  S02 * (P_01 * S01 - P_00 * S11) + P_02 * (S01 * S01) -
                  P_02 * S00 * S11) *
                invPart;
    float K10 = -(S02 * (P_12 * S11 - P_11 * S12) -
                  S01 * (P_12 * S12 - P_11 * S22) + P_01 * (S12 * S12) -
                  P_01 * S11 * S22) *
                invPart;
    float K11 = -(S12 * (P_12 * S00 - P_01 * S02) -
                  S01 * (P_12 * S02 - P_01 * S22) + P_11 * (S02 * S02) -
                  P_11 * S00 * S22) *
                invPart;
    float K12 =
            (S12 * (P_01 * S01 - P_11 * S00) + S02 * (P_11 * S01 - P_01 * S11) -
             P_12 * (S01 * S01) + P_12 * S00 * S11) *
            invPart;
    float K20 =
            (S02 * (P_12 * S12 - P_22 * S11) + S01 * (P_22 * S12 - P_12 * S22) -
             P_02 * (S12 * S12) + P_02 * S11 * S22) *
            invPart;
    float K21 =
            (S12 * (P_02 * S02 - P_22 * S00) + S01 * (P_22 * S02 - P_02 * S22) -
             P_12 * (S02 * S02) + P_12 * S00 * S22) *
            invPart;
    float K22 =
            (S12 * (P_02 * S01 - P_12 * S00) + S02 * (P_12 * S01 - P_02 * S11) -
             P_22 * (S01 * S01) + P_22 * S00 * S11) *
            invPart;
    float K30 =
            (S02 * (P_13 * S12 - P_23 * S11) + S01 * (P_23 * S12 - P_13 * S22) -
             P_03 * (S12 * S12) + P_03 * S11 * S22) *
            invPart;
    float K31 =
            (S12 * (P_03 * S02 - P_23 * S00) + S01 * (P_23 * S02 - P_03 * S22) -
             P_13 * (S02 * S02) + P_13 * S00 * S22) *
            invPart;
    float K32 =
            (S12 * (P_03 * S01 - P_13 * S00) + S02 * (P_13 * S01 - P_03 * S11) -
             P_23 * (S01 * S01) + P_23 * S00 * S11) *
            invPart;
    float K40 =
            (S02 * (P_14 * S12 - P_24 * S11) + S01 * (P_24 * S12 - P_14 * S22) -
             P_04 * (S12 * S12) + P_04 * S11 * S22) *
            invPart;
    float K41 =
            (S12 * (P_04 * S02 - P_24 * S00) + S01 * (P_24 * S02 - P_04 * S22) -
             P_14 * (S02 * S02) + P_14 * S00 * S22) *
            invPart;
    float K42 =
            (S12 * (P_04 * S01 - P_14 * S00) + S02 * (P_14 * S01 - P_04 * S11) -
             P_24 * (S01 * S01) + P_24 * S00 * S11) *
            invPart;
    float K50 =
            (S02 * (P_15 * S12 - P_25 * S11) + S01 * (P_25 * S12 - P_15 * S22) -
             P_05 * (S12 * S12) + P_05 * S11 * S22) *
            invPart;
    float K51 =
            (S12 * (P_05 * S02 - P_25 * S00) + S01 * (P_25 * S02 - P_05 * S22) -
             P_15 * (S02 * S02) + P_15 * S00 * S22) *
            invPart;
    float K52 =
            (S12 * (P_05 * S01 - P_15 * S00) + S02 * (P_15 * S01 - P_05 * S11) -
             P_25 * (S01 * S01) + P_25 * S00 * S11) *
            invPart;

    // update a posteriori
    x0_ = x_0 + K00 * y0 + K01 * y1 + K02 * y2;
    x1_ = x_1 + K10 * y0 + K11 * y1 + K12 * y2;
    x2_ = x_2 + K20 * y0 + K21 * y1 + K22 * y2;

#if AUTO_STATIC_DETECTTION == 0
    x3_ = x_3 + K30 * y0 + K31 * y1 + K32 * y2;
    x3_ = CLAMP(x3_, GYRO_BIAS_MAX);
    //	//x3_ = CLAMP((x_3 + K30*y0 + K31*y1 + K32*y2), GYRO_BIAS_MAX);
    x4_ = x_4 + K40 * y0 + K41 * y1 + K42 * y2;
    x4_ = CLAMP(x4_, GYRO_BIAS_MAX);
    //	//x4_ = CLAMP((x_4 + K40*y0 + K41*y1 + K42*y2), GYRO_BIAS_MAX);
    x5_ = x_5 + K50 * y0 + K51 * y1 + K52 * y2;
    x5_ = CLAMP(x5_, GYRO_BIAS_MAX);
//	//x5_ = CLAMP((x_5 + K50*y0 + K51*y1 + K52*y2), GYRO_BIAS_MAX);
#else
    x3_ = 0.0f;
    x4_ = 0.0f;
    x5_ = 0.0f;
#endif

    // update a posteriori covariance
    float K00_1 = K00 - 1.0f;
    float K11_1 = K11 - 1.0f;
    float K22_1 = K22 - 1.0f;

    float common1 = P_01 * K00_1 + K01 * P_11 + K02 * P_12;
    float common2 = P_02 * K00_1 + K01 * P_12 + K02 * P_22;
    float common3 = P_00 * K00_1 + K01 * P_01 + K02 * P_02;
    float common4 = P_01 * K11_1 + K10 * P_00 + K12 * P_02;
    float common5 = P_12 * K11_1 + K10 * P_02 + K12 * P_22;
    float common6 = P_11 * K11_1 + K10 * P_01 + K12 * P_12;
    float common7 = P_02 * K22_1 + K20 * P_00 + K21 * P_01;
    float common8 = P_12 * K22_1 + K20 * P_01 + K21 * P_11;
    float common9 = P_22 * K22_1 + K20 * P_02 + K21 * P_12;
    float commonA = -P_03 + K30 * P_00 + K31 * P_01 + K32 * P_02;
    float commonB = -P_13 + K30 * P_01 + K31 * P_11 + K32 * P_12;
    float commonC = -P_23 + K30 * P_02 + K31 * P_12 + K32 * P_22;

    float P__00 = K01 * common1 + K02 * common2 + (K00 * K00) * r_adab +
                  (K01 * K01) * r_adab + (K02 * K02) * r_adab + K00_1 * common3;
    float P__01 = K10 * common3 + K12 * common2 + K11_1 * common1 +
                  K00 * K10 * r_adab + K01 * K11 * r_adab + K02 * K12 * r_adab;
    float P__02 = K20 * common3 + K21 * common1 + K22_1 * common2 +
                  K00 * K20 * r_adab + K01 * K21 * r_adab + K02 * K22 * r_adab;
    float P__03 = -P_03 * K00_1 + K30 * common3 + K31 * common1 +
                  K32 * common2 - K01 * P_13 - K02 * P_23 + K00 * K30 * r_adab +
                  K01 * K31 * r_adab + K02 * K32 * r_adab;
    float P__04 = -P_04 * K00_1 + K40 * common3 + K41 * common1 +
                  K42 * common2 - K01 * P_14 - K02 * P_24 + K00 * K40 * r_adab +
                  K01 * K41 * r_adab + K02 * K42 * r_adab;
    float P__05 = -P_05 * K00_1 + K50 * common3 + K51 * common1 +
                  K52 * common2 - K01 * P_15 - K02 * P_25 + K00 * K50 * r_adab +
                  K01 * K51 * r_adab + K02 * K52 * r_adab;
    float P__11 = K10 * common4 + K12 * common5 + (K10 * K10) * r_adab +
                  (K11 * K11) * r_adab + (K12 * K12) * r_adab + K11_1 * common6;
    float P__12 = K20 * common4 + K21 * common6 + K22_1 * common5 +
                  K10 * K20 * r_adab + K11 * K21 * r_adab + K12 * K22 * r_adab;
    float P__13 = -P_13 * K11_1 + K30 * common4 + K31 * common6 +
                  K32 * common5 - K10 * P_03 - K12 * P_23 + K10 * K30 * r_adab +
                  K11 * K31 * r_adab + K12 * K32 * r_adab;
    float P__14 = -P_14 * K11_1 + K40 * common4 + K41 * common6 +
                  K42 * common5 - K10 * P_04 - K12 * P_24 + K10 * K40 * r_adab +
                  K11 * K41 * r_adab + K12 * K42 * r_adab;
    float P__15 = -P_15 * K11_1 + K50 * common4 + K51 * common6 +
                  K52 * common5 - K10 * P_05 - K12 * P_25 + K10 * K50 * r_adab +
                  K11 * K51 * r_adab + K12 * K52 * r_adab;
    float P__22 = K20 * common7 + K21 * common8 + (K20 * K20) * r_adab +
                  (K21 * K21) * r_adab + (K22 * K22) * r_adab + K22_1 * common9;
    float P__23 = -P_23 * K22_1 + K30 * common7 + K31 * common8 +
                  K32 * common9 - K20 * P_03 - K21 * P_13 + K20 * K30 * r_adab +
                  K21 * K31 * r_adab + K22 * K32 * r_adab;
    float P__24 = -P_24 * K22_1 + K40 * common7 + K41 * common8 +
                  K42 * common9 - K20 * P_04 - K21 * P_14 + K20 * K40 * r_adab +
                  K21 * K41 * r_adab + K22 * K42 * r_adab;
    float P__25 = -P_25 * K22_1 + K50 * common7 + K51 * common8 +
                  K52 * common9 - K20 * P_05 - K21 * P_15 + K20 * K50 * r_adab +
                  K21 * K51 * r_adab + K22 * K52 * r_adab;
    float P__33 = P_33 + (K30 * K30) * r_adab + (K31 * K31) * r_adab +
                  (K32 * K32) * r_adab + K30 * commonA + K31 * commonB +
                  K32 * commonC - K30 * P_03 - K31 * P_13 - K32 * P_23;
    float P__34 = P_34 + K40 * commonA + K41 * commonB + K42 * commonC -
                  K30 * P_04 - K31 * P_14 - K32 * P_24 + K30 * K40 * r_adab +
                  K31 * K41 * r_adab + K32 * K42 * r_adab;
    float P__35 = P_35 + K50 * commonA + K51 * commonB + K52 * commonC -
                  K30 * P_05 - K31 * P_15 - K32 * P_25 + K30 * K50 * r_adab +
                  K31 * K51 * r_adab + K32 * K52 * r_adab;
    float P__44 = P_44 + (K40 * K40) * r_adab + (K41 * K41) * r_adab +
                  (K42 * K42) * r_adab +
                  K40 * (-P_04 + K40 * P_00 + K41 * P_01 + K42 * P_02) +
                  K41 * (-P_14 + K40 * P_01 + K41 * P_11 + K42 * P_12) +
                  K42 * (-P_24 + K40 * P_02 + K41 * P_12 + K42 * P_22) -
                  K40 * P_04 - K41 * P_14 - K42 * P_24;
    float P__45 = P_45 + K50 * (-P_04 + K40 * P_00 + K41 * P_01 + K42 * P_02) +
                  K51 * (-P_14 + K40 * P_01 + K41 * P_11 + K42 * P_12) +
                  K52 * (-P_24 + K40 * P_02 + K41 * P_12 + K42 * P_22) -
                  K40 * P_05 - K41 * P_15 - K42 * P_25 + K40 * K50 * r_adab +
                  K41 * K51 * r_adab + K42 * K52 * r_adab;
    float P__55 = P_55 + (K50 * K50) * r_adab + (K51 * K51) * r_adab +
                  (K52 * K52) * r_adab +
                  K50 * (-P_05 + K50 * P_00 + K51 * P_01 + K52 * P_02) +
                  K51 * (-P_15 + K50 * P_01 + K51 * P_11 + K52 * P_12) +
                  K52 * (-P_25 + K50 * P_02 + K51 * P_12 + K52 * P_22) -
                  K50 * P_05 - K51 * P_15 - K52 * P_25;

    // Normalization of covariance
    float inv_len = invSqrt(x0_ * x0_ + x1_ * x1_ + x2_ * x2_);
    float invlen3 = inv_len * inv_len * inv_len;
    float invlen32 = (invlen3 * invlen3);

    float x1x1_x2x2 = (x1_ * x1_ + x2_ * x2_);
    float x0x0_x2x2 = (x0_ * x0_ + x2_ * x2_);
    float x0x0_x1x1 = (x0_ * x0_ + x1_ * x1_);

    P00_ = invlen32 * (-x1x1_x2x2 * (-P__00 * x1x1_x2x2 + P__01 * x0_ * x1_ +
                                     P__02 * x0_ * x2_) +
                       x0_ * x1_ *
                               (-P__01 * x1x1_x2x2 + P__11 * x0_ * x1_ +
                                P__12 * x0_ * x2_) +
                       x0_ * x2_ *
                               (-P__02 * x1x1_x2x2 + P__12 * x0_ * x1_ +
                                P__22 * x0_ * x2_));
    P01_ = invlen32 * (-x0x0_x2x2 * (-P__01 * x1x1_x2x2 + P__11 * x0_ * x1_ +
                                     P__12 * x0_ * x2_) +
                       x0_ * x1_ *
                               (-P__00 * x1x1_x2x2 + P__01 * x0_ * x1_ +
                                P__02 * x0_ * x2_) +
                       x1_ * x2_ *
                               (-P__02 * x1x1_x2x2 + P__12 * x0_ * x1_ +
                                P__22 * x0_ * x2_));
    P02_ = invlen32 * (-x0x0_x1x1 * (-P__02 * x1x1_x2x2 + P__12 * x0_ * x1_ +
                                     P__22 * x0_ * x2_) +
                       x0_ * x2_ *
                               (-P__00 * x1x1_x2x2 + P__01 * x0_ * x1_ +
                                P__02 * x0_ * x2_) +
                       x1_ * x2_ *
                               (-P__01 * x1x1_x2x2 + P__11 * x0_ * x1_ +
                                P__12 * x0_ * x2_));
    P03_ = -invlen3 *
           (-P__03 * x1x1_x2x2 + P__13 * x0_ * x1_ + P__23 * x0_ * x2_);
    P04_ = -invlen3 *
           (-P__04 * x1x1_x2x2 + P__14 * x0_ * x1_ + P__24 * x0_ * x2_);
    P05_ = -invlen3 *
           (-P__05 * x1x1_x2x2 + P__15 * x0_ * x1_ + P__25 * x0_ * x2_);
    P11_ = invlen32 * (-x0x0_x2x2 * (-P__11 * x0x0_x2x2 + P__01 * x0_ * x1_ +
                                     P__12 * x1_ * x2_) +
                       x0_ * x1_ *
                               (-P__01 * x0x0_x2x2 + P__00 * x0_ * x1_ +
                                P__02 * x1_ * x2_) +
                       x1_ * x2_ *
                               (-P__12 * x0x0_x2x2 + P__02 * x0_ * x1_ +
                                P__22 * x1_ * x2_));
    P12_ = invlen32 * (-x0x0_x1x1 * (-P__12 * x0x0_x2x2 + P__02 * x0_ * x1_ +
                                     P__22 * x1_ * x2_) +
                       x0_ * x2_ *
                               (-P__01 * x0x0_x2x2 + P__00 * x0_ * x1_ +
                                P__02 * x1_ * x2_) +
                       x1_ * x2_ *
                               (-P__11 * x0x0_x2x2 + P__01 * x0_ * x1_ +
                                P__12 * x1_ * x2_));
    P13_ = -invlen3 *
           (-P__13 * x0x0_x2x2 + P__03 * x0_ * x1_ + P__23 * x1_ * x2_);
    P14_ = -invlen3 *
           (-P__14 * x0x0_x2x2 + P__04 * x0_ * x1_ + P__24 * x1_ * x2_);
    P15_ = -invlen3 *
           (-P__15 * x0x0_x2x2 + P__05 * x0_ * x1_ + P__25 * x1_ * x2_);
    P22_ = invlen32 * (-x0x0_x1x1 * (-P__22 * x0x0_x1x1 + P__02 * x0_ * x2_ +
                                     P__12 * x1_ * x2_) +
                       x0_ * x2_ *
                               (-P__02 * x0x0_x1x1 + P__00 * x0_ * x2_ +
                                P__01 * x1_ * x2_) +
                       x1_ * x2_ *
                               (-P__12 * x0x0_x1x1 + P__01 * x0_ * x2_ +
                                P__11 * x1_ * x2_));
    P23_ = -invlen3 *
           (-P__23 * x0x0_x1x1 + P__03 * x0_ * x2_ + P__13 * x1_ * x2_);
    P24_ = -invlen3 *
           (-P__24 * x0x0_x1x1 + P__04 * x0_ * x2_ + P__14 * x1_ * x2_);
    P25_ = -invlen3 *
           (-P__25 * x0x0_x1x1 + P__05 * x0_ * x2_ + P__15 * x1_ * x2_);
    P33_ = P__33;
    P34_ = P__34;
    P35_ = P__35;
    P44_ = P__44;
    P45_ = P__45;
    P55_ = P__55;

    // increment covariance slightly at each iteration (nonoptimal but keeps the
    // filter stable against rounding errors in 32bit float computation)
    P00_ += VARIANCE_SAFETY_INCREMENT;
    P11_ += VARIANCE_SAFETY_INCREMENT;
    P22_ += VARIANCE_SAFETY_INCREMENT;
    //	P33_ += VARIANCE_SAFETY_INCREMENT;
    //	P44_ += VARIANCE_SAFETY_INCREMENT;
    //	P55_ += VARIANCE_SAFETY_INCREMENT;

    // variance is required to be always at least the minimum value
    if (P00_ < VARIANCE_MIN_LIMIT)
        P00_ = VARIANCE_MIN_LIMIT;
    if (P11_ < VARIANCE_MIN_LIMIT)
        P11_ = VARIANCE_MIN_LIMIT;
    if (P22_ < VARIANCE_MIN_LIMIT)
        P22_ = VARIANCE_MIN_LIMIT;
    if (P33_ < VARIANCE_MIN_LIMIT)
        P33_ = VARIANCE_MIN_LIMIT;
    if (P44_ < VARIANCE_MIN_LIMIT)
        P44_ = VARIANCE_MIN_LIMIT;
    if (P55_ < VARIANCE_MIN_LIMIT)
        P55_ = VARIANCE_MIN_LIMIT;

    // normalized a posteriori state
    x0_ = x0_ * inv_len;
    x1_ = x1_ * inv_len;
    x2_ = x2_ * inv_len;

    float u_nb0 = u0 - x3_;
    float u_nb1 = u1 - x4_;
    float u_nb2 = u2 - x5_;

    // calculate the second row (sr) from a rotated first row (rotation with bias
    // corrected gyroscope measurement)
    sr0_ = -fr1_ * x_last_[2] + fr2_ * x_last_[1] -
           dt_ * (x_last_[1] * (fr1_ * u_nb0 - fr0_ * u_nb1) +
                  x_last_[2] * (fr2_ * u_nb0 - fr0_ * u_nb2));
    sr1_ = fr0_ * x_last_[2] - fr2_ * x_last_[0] +
           dt_ * (x_last_[0] * (fr1_ * u_nb0 - fr0_ * u_nb1) -
                  x_last_[2] * (fr2_ * u_nb1 - fr1_ * u_nb2));
    sr2_ = -fr0_ * x_last_[1] + fr1_ * x_last_[0] +
           dt_ * (x_last_[0] * (fr2_ * u_nb0 - fr0_ * u_nb2) +
                  x_last_[1] * (fr2_ * u_nb1 - fr1_ * u_nb2));

    // save the estimated non-gravitational acceleration
    linear_a_[0] = (z0 - x0_) * g_;
    linear_a_[1] = (z1 - x1_) * g_;
    linear_a_[2] = (z2 - x2_) * g_;
}

void DCM_AHRS::computeAngles(void)
{
    // alternative method estimating the whole rotation matrix
    // integrate full rotation matrix (using first row estimate in memory)
    // normalize the second row
    float invlen = invSqrt(sr0_ * sr0_ + sr1_ * sr1_ + sr2_ * sr2_);
    sr0_ *= invlen;
    sr1_ *= invlen;
    sr2_ *= invlen;

    // recompute the first row (ensure perpendicularity)
    fr0_ = sr1_ * x_last_[2] - sr2_ * x_last_[1];
    fr1_ = -sr0_ * x_last_[2] + sr2_ * x_last_[0];
    fr2_ = sr0_ * x_last_[1] - sr1_ * x_last_[0];

    // normalize the first row
    invlen = invSqrt(fr0_ * fr0_ + fr1_ * fr1_ + fr2_ * fr2_);
    fr0_ *= invlen;
    fr1_ *= invlen;
    fr2_ *= invlen;

    arm_atan2_f32(sr0_, fr0_, &edata_.yaw);
    arm_atan2_f32(x1_, x2_, &edata_.roll);
    edata_.pitch = asinf(-x0_);
}

void DCM_AHRS::update(float _gx, float _gy, float _gz, float _ax, float _ay,
                      float _az, float _dt)
{
    updateDCM(_gx, _gy, _gz, _ax, _ay, _az, _dt);

    // compute the angles from the current state
    computeAngles();
}

edata_s DCM_AHRS::getEdata(void) { return edata_; }

float DCM_AHRS::getRoll(void) { return edata_.roll; }

float DCM_AHRS::getPitch(void) { return edata_.pitch; }

float DCM_AHRS::getYaw(void) { return edata_.yaw; }

void DCM_AHRS::getQuaternion(float *q)
{
    q[0] = x0_;
    q[1] = x1_;
    q[2] = x2_;
    q[3] = x3_;
}

float DCM_AHRS::invSqrt(float x)
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

}
