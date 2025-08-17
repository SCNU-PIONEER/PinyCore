#include "DcmAHRS.hpp"
#include "dsp/fast_math_functions.h"

#define AUTO_STATIC_DETECTTION 1

namespace IMU_DCM_AHRS {

void DcmAhrs::init()
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

void DcmAhrs::updateDCM(float _gx, float _gy, float _gz, float _ax, float _ay,
                        float _az, float _dt)
{
    dt_ = _dt;
    float invG0 = (1.f / g_);
    float invG02 = (invG0 * invG0);
    // save last state to memory for rotation estimation
    x_last_[0] = x0_;
    x_last_[1] = x1_;
    x_last_[2] = x2_;

    // control input (gyroscopes)
    float u0 = 0.0f, u1 = 0.0f, u2 = 0.0f;

    // control input (gyroscopes)
    u0 = _gx;
    u1 = _gy;
    u2 = -_gz;

    // state prediction
    float x0 = x0_ - (dt_ * (u1 * x2_ - u2 * x1_ + x1_ * x5_ - x2_ * x4_));
    float x1 = x1_ + (dt_ * (u0 * x2_ - u2 * x0_ + x0_ * x5_ - x2_ * x3_));
    float x2 = x2_ - (dt_ * (u0 * x1_ - u1 * x0_ + x0_ * x4_ - x1_ * x3_));
#if AUTO_STATIC_DETECTTION == 0
    float x3 = x3_;
    float x4 = x4_;
    float x5 = x5_;
#endif

    // covariance prediction
    float dt2 = dt_ * dt_;
    float mP00 = P00_ -
                 (dt_ * (P05_ * x1_ * 2.0f - P04_ * x2_ * 2.0f +
                         P02_ * (u1 - x4_) * 2.0f - P01_ * (u2 - x5_) * 2.0f)) -
                 (dt2 * (-DCMVariance_ +
                         x2_ * (P45_ * x1_ - P44_ * x2_ + P24_ * (u1 - x4_) -
                                P14_ * (u2 - x5_)) +
                         x1_ * (P45_ * x2_ - P55_ * x1_ - P25_ * (u1 - x4_) +
                                P15_ * (u2 - x5_)) +
                         (u2 - x5_) * (P15_ * x1_ - P14_ * x2_ +
                                       P12_ * (u1 - x4_) - P11_ * (u2 - x5_)) -
                         (u1 - x4_) * (P25_ * x1_ - P24_ * x2_ +
                                       P22_ * (u1 - x4_) - P12_ * (u2 - x5_))));
    float mP01 = P01_ +
                 (dt_ * (P05_ * x0_ - P03_ * x2_ - P15_ * x1_ + P14_ * x2_ +
                         P02_ * (u0 - x3_) - P12_ * (u1 - x4_) -
                         P00_ * (u2 - x5_) + P11_ * (u2 - x5_))) +
                 (dt2 * (x2_ * (P35_ * x1_ - P34_ * x2_ + P23_ * (u1 - x4_) -
                                P13_ * (u2 - x5_)) +
                         x0_ * (P45_ * x2_ - P55_ * x1_ - P25_ * (u1 - x4_) +
                                P15_ * (u2 - x5_)) +
                         (u2 - x5_) * (P05_ * x1_ - P04_ * x2_ +
                                       P02_ * (u1 - x4_) - P01_ * (u2 - x5_)) -
                         (u0 - x3_) * (P25_ * x1_ - P24_ * x2_ +
                                       P22_ * (u1 - x4_) - P12_ * (u2 - x5_))));
    float mP02 = P02_ -
                 (dt_ * (P04_ * x0_ - P03_ * x1_ + P25_ * x1_ - P24_ * x2_ +
                         P01_ * (u0 - x3_) - P00_ * (u1 - x4_) +
                         P22_ * (u1 - x4_) - P12_ * (u2 - x5_))) -
                 (dt2 * (x1_ * (P35_ * x1_ - P34_ * x2_ + P23_ * (u1 - x4_) -
                                P13_ * (u2 - x5_)) -
                         x0_ * (P45_ * x1_ - P44_ * x2_ + P24_ * (u1 - x4_) -
                                P14_ * (u2 - x5_)) +
                         (u1 - x4_) * (P05_ * x1_ - P04_ * x2_ +
                                       P02_ * (u1 - x4_) - P01_ * (u2 - x5_)) -
                         (u0 - x3_) * (P15_ * x1_ - P14_ * x2_ +
                                       P12_ * (u1 - x4_) - P11_ * (u2 - x5_))));
    float mP03 = P03_ - (dt_ * (P35_ * x1_ - P34_ * x2_ + P23_ * (u1 - x4_) -
                                P13_ * (u2 - x5_)));
    float mP04 = P04_ - (dt_ * (P45_ * x1_ - P44_ * x2_ + P24_ * (u1 - x4_) -
                                P14_ * (u2 - x5_)));
    float mP05 = P05_ + (dt_ * (P45_ * x2_ - P55_ * x1_ - P25_ * (u1 - x4_) +
                                P15_ * (u2 - x5_)));
    float mP11 = P11_ +
                 (dt_ * (P15_ * x0_ * 2.0f - P13_ * x2_ * 2.0f +
                         P12_ * (u0 - x3_) * 2.0f - P01_ * (u2 - x5_) * 2.0f)) -
                 (dt2 * (-DCMVariance_ +
                         x2_ * (P35_ * x0_ - P33_ * x2_ + P23_ * (u0 - x3_) -
                                P03_ * (u2 - x5_)) +
                         x0_ * (P35_ * x2_ - P55_ * x0_ - P25_ * (u0 - x3_) +
                                P05_ * (u2 - x5_)) +
                         (u2 - x5_) * (P05_ * x0_ - P03_ * x2_ +
                                       P02_ * (u0 - x3_) - P00_ * (u2 - x5_)) -
                         (u0 - x3_) * (P25_ * x0_ - P23_ * x2_ +
                                       P22_ * (u0 - x3_) - P02_ * (u2 - x5_))));
    float mP12 = P12_ -
                 (dt_ * (P14_ * x0_ - P13_ * x1_ - P25_ * x0_ + P23_ * x2_ +
                         P11_ * (u0 - x3_) - P01_ * (u1 - x4_) -
                         P22_ * (u0 - x3_) + P02_ * (u2 - x5_))) +
                 (dt2 * (x1_ * (P35_ * x0_ - P33_ * x2_ + P23_ * (u0 - x3_) -
                                P03_ * (u2 - x5_)) -
                         x0_ * (P45_ * x0_ - P34_ * x2_ + P24_ * (u0 - x3_) -
                                P04_ * (u2 - x5_)) +
                         (u1 - x4_) * (P05_ * x0_ - P03_ * x2_ +
                                       P02_ * (u0 - x3_) - P00_ * (u2 - x5_)) -
                         (u0 - x3_) * (P15_ * x0_ - P13_ * x2_ +
                                       P12_ * (u0 - x3_) - P01_ * (u2 - x5_))));
    float mP13 = P13_ + (dt_ * (P35_ * x0_ - P33_ * x2_ + P23_ * (u0 - x3_) -
                                P03_ * (u2 - x5_)));
    float mP14 = P14_ + (dt_ * (P45_ * x0_ - P34_ * x2_ + P24_ * (u0 - x3_) -
                                P04_ * (u2 - x5_)));
    float mP15 = P15_ - (dt_ * (P35_ * x2_ - P55_ * x0_ - P25_ * (u0 - x3_) +
                                P05_ * (u2 - x5_)));
    float mP22 = P22_ -
                 (dt_ * (P24_ * x0_ * 2.0f - P23_ * x1_ * 2.0f +
                         P12_ * (u0 - x3_) * 2.0f - P02_ * (u1 - x4_) * 2.0f)) -
                 (dt2 * (-DCMVariance_ +
                         x1_ * (P34_ * x0_ - P33_ * x1_ + P13_ * (u0 - x3_) -
                                P03_ * (u1 - x4_)) +
                         x0_ * (P34_ * x1_ - P44_ * x0_ - P14_ * (u0 - x3_) +
                                P04_ * (u1 - x4_)) +
                         (u1 - x4_) * (P04_ * x0_ - P03_ * x1_ +
                                       P01_ * (u0 - x3_) - P00_ * (u1 - x4_)) -
                         (u0 - x3_) * (P14_ * x0_ - P13_ * x1_ +
                                       P11_ * (u0 - x3_) - P01_ * (u1 - x4_))));
    float mP23 = P23_ - (dt_ * (P34_ * x0_ - P33_ * x1_ + P13_ * (u0 - x3_) -
                                P03_ * (u1 - x4_)));
    float mP24 = P24_ + (dt_ * (P34_ * x1_ - P44_ * x0_ - P14_ * (u0 - x3_) +
                                P04_ * (u1 - x4_)));
    float mP25 = P25_ + (dt_ * (P35_ * x1_ - P45_ * x0_ - P15_ * (u0 - x3_) +
                                P05_ * (u1 - x4_)));
    float mP33 = P33_ + (dt2 * biasVariance_);
    float mP34 = P34_;
    float mP35 = P35_;
    float mP44 = P44_ + (dt2 * biasVariance_);
    float mP45 = P45_;
    float mP55 = P55_ + (dt2 * biasVariance_);

    // measurements (accelerometers)
    float z0 = _ax * invG0;
    float z1 = _ay * invG0;
    float z2 = -_az * invG0;

    // Kalman innovation
    float y0 = z0 - x0;
    float y1 = z1 - x1;
    float y2 = z2 - x2;

    float aLen = sqrtf((y0 * y0) + (y1 * y1) + (y2 * y2)) * g_;
    float rAdab =
            (measurementVariance_ + aLen * measurementVarianceVariableGain_) *
            invG02;

    // innovation covariance
    float mS00 = mP00 + rAdab;
    float mS01 = mP01;
    float mS02 = mP02;
    float mS11 = mP11 + rAdab;
    float mS12 = mP12;
    float mS22 = mP22 + rAdab;

    // verify that the innovation covariance is large enough
    if (mS00 < VARIANCE_MIN_LIMIT)
        mS00 = VARIANCE_MIN_LIMIT;
    if (mS11 < VARIANCE_MIN_LIMIT)
        mS11 = VARIANCE_MIN_LIMIT;
    if (mS22 < VARIANCE_MIN_LIMIT)
        mS22 = VARIANCE_MIN_LIMIT;

    // determinant of S
    float detS = (-mS00 * (mS12 * mS12)) - ((mS02 * mS02) * mS11) -
                 ((mS01 * mS01) * mS22) + (mS01 * mS02 * mS12 * 2.0f) +
                 (mS00 * mS11 * mS22);
    if (detS == 0.0f)
        return;

    // Kalman gain
    float invPart = 1.0f / detS;
    float mK00 = -((mS02 * (mP02 * mS11 - mP01 * mS12)) -
                   (mS01 * (mP02 * mS12 - mP01 * mS22)) +
                   (mP00 * (mS12 * mS12)) - (mP00 * mS11 * mS22)) *
                 invPart;
    float mK01 = -((mS12 * (mP02 * mS00 - mP00 * mS02)) -
                   (mS01 * (mP02 * mS02 - mP00 * mS22)) +
                   (mP01 * (mS02 * mS02)) - (mP01 * mS00 * mS22)) *
                 invPart;
    float mK02 = -((mS12 * (mP01 * mS00 - mP00 * mS01)) -
                   (mS02 * (mP01 * mS01 - mP00 * mS11)) +
                   (mP02 * (mS01 * mS01)) - (mP02 * mS00 * mS11)) *
                 invPart;
    float mK10 = -((mS02 * (mP12 * mS11 - mP11 * mS12)) -
                   (mS01 * (mP12 * mS12 - mP11 * mS22)) +
                   (mP01 * (mS12 * mS12)) - (mP01 * mS11 * mS22)) *
                 invPart;
    float mK11 = -((mS12 * (mP12 * mS00 - mP01 * mS02)) -
                   (mS01 * (mP12 * mS02 - mP01 * mS22)) +
                   (mP11 * (mS02 * mS02)) - (mP11 * mS00 * mS22)) *
                 invPart;
    float mK12 = (mS12 * (mP01 * mS01 - mP11 * mS00) +
                  mS02 * (mP11 * mS01 - mP01 * mS11) - mP12 * (mS01 * mS01) +
                  mP12 * mS00 * mS11) *
                 invPart;
    float mK20 = (mS02 * (mP12 * mS12 - mP22 * mS11) +
                  mS01 * (mP22 * mS12 - mP12 * mS22) - mP02 * (mS12 * mS12) +
                  mP02 * mS11 * mS22) *
                 invPart;
    float mK21 = (mS12 * (mP02 * mS02 - mP22 * mS00) +
                  mS01 * (mP22 * mS02 - mP02 * mS22) - mP12 * (mS02 * mS02) +
                  mP12 * mS00 * mS22) *
                 invPart;
    float mK22 = (mS12 * (mP02 * mS01 - mP12 * mS00) +
                  mS02 * (mP12 * mS01 - mP02 * mS11) - mP22 * (mS01 * mS01) +
                  mP22 * mS00 * mS11) *
                 invPart;
    float mK30 = (mS02 * (mP13 * mS12 - mP23 * mS11) +
                  mS01 * (mP23 * mS12 - mP13 * mS22) - mP03 * (mS12 * mS12) +
                  mP03 * mS11 * mS22) *
                 invPart;
    float mK31 = (mS12 * (mP03 * mS02 - mP23 * mS00) +
                  mS01 * (mP23 * mS02 - mP03 * mS22) - mP13 * (mS02 * mS02) +
                  mP13 * mS00 * mS22) *
                 invPart;
    float mK32 = (mS12 * (mP03 * mS01 - mP13 * mS00) +
                  mS02 * (mP13 * mS01 - mP03 * mS11) - mP23 * (mS01 * mS01) +
                  mP23 * mS00 * mS11) *
                 invPart;
    float mK40 = (mS02 * (mP14 * mS12 - mP24 * mS11) +
                  mS01 * (mP24 * mS12 - mP14 * mS22) - mP04 * (mS12 * mS12) +
                  mP04 * mS11 * mS22) *
                 invPart;
    float mK41 = (mS12 * (mP04 * mS02 - mP24 * mS00) +
                  mS01 * (mP24 * mS02 - mP04 * mS22) - mP14 * (mS02 * mS02) +
                  mP14 * mS00 * mS22) *
                 invPart;
    float mK42 = (mS12 * (mP04 * mS01 - mP14 * mS00) +
                  mS02 * (mP14 * mS01 - mP04 * mS11) - mP24 * (mS01 * mS01) +
                  mP24 * mS00 * mS11) *
                 invPart;
    float mK50 = (mS02 * (mP15 * mS12 - mP25 * mS11) +
                  mS01 * (mP25 * mS12 - mP15 * mS22) - mP05 * (mS12 * mS12) +
                  mP05 * mS11 * mS22) *
                 invPart;
    float mK51 = (mS12 * (mP05 * mS02 - mP25 * mS00) +
                  mS01 * (mP25 * mS02 - mP05 * mS22) - mP15 * (mS02 * mS02) +
                  mP15 * mS00 * mS22) *
                 invPart;
    float mK52 = (mS12 * (mP05 * mS01 - mP15 * mS00) +
                  mS02 * (mP15 * mS01 - mP05 * mS11) - mP25 * (mS01 * mS01) +
                  mP25 * mS00 * mS11) *
                 invPart;

    // update a posteriori
    x0_ = x0 + mK00 * y0 + mK01 * y1 + mK02 * y2;
    x1_ = x1 + mK10 * y0 + mK11 * y1 + mK12 * y2;
    x2_ = x2 + mK20 * y0 + mK21 * y1 + mK22 * y2;

#if AUTO_STATIC_DETECTTION == 0
    x3_ = x3 + mK30 * y0 + mK31 * y1 + mK32 * y2;
    x3_ = clamp(x3_, GYRO_BIAS_MAX);
    //	//x3_ = clamp((x3 + mK30*y0 + mK31*y1 + mK32*y2), GYRO_BIAS_MAX);
    x4_ = x4 + mK40 * y0 + mK41 * y1 + mK42 * y2;
    x4_ = clamp(x4_, GYRO_BIAS_MAX);
    //	//x4_ = clamp((x4 + mK40*y0 + mK41*y1 + mK42*y2), GYRO_BIAS_MAX);
    x5_ = x5 + mK50 * y0 + mK51 * y1 + mK52 * y2;
    x5_ = clamp(x5_, GYRO_BIAS_MAX);
//	//x5_ = clamp((x5 + mK50*y0 + mK51*y1 + mK52*y2), GYRO_BIAS_MAX);
#else
    x3_ = 0.0f;
    x4_ = 0.0f;
    x5_ = 0.0f;
#endif

    // update a posteriori covariance
    float mK001 = mK00 - 1.0f;
    float mK111 = mK11 - 1.0f;
    float mK221 = mK22 - 1.0f;

    float common1 = (mP01 * mK001) + (mK01 * mP11) + (mK02 * mP12);
    float common2 = (mP02 * mK001) + (mK01 * mP12) + (mK02 * mP22);
    float common3 = (mP00 * mK001) + (mK01 * mP01) + (mK02 * mP02);
    float common4 = (mP01 * mK111) + (mK10 * mP00) + (mK12 * mP02);
    float common5 = (mP12 * mK111) + (mK10 * mP02) + (mK12 * mP22);
    float common6 = (mP11 * mK111) + (mK10 * mP01) + (mK12 * mP12);
    float common7 = (mP02 * mK221) + (mK20 * mP00) + (mK21 * mP01);
    float common8 = (mP12 * mK221) + (mK20 * mP01) + (mK21 * mP11);
    float common9 = (mP22 * mK221) + (mK20 * mP02) + (mK21 * mP12);
    float commonA = -mP03 + (mK30 * mP00) + (mK31 * mP01) + (mK32 * mP02);
    float commonB = -mP13 + (mK30 * mP01) + (mK31 * mP11) + (mK32 * mP12);
    float commonC = -mP23 + (mK30 * mP02) + (mK31 * mP12) + (mK32 * mP22);

    float mPP00 = (mK01 * common1) + (mK02 * common2) +
                  ((mK00 * mK00) * rAdab) + ((mK01 * mK01) * rAdab) +
                  ((mK02 * mK02) * rAdab) + (mK001 * common3);
    float mPP01 = (mK10 * common3) + (mK12 * common2) + (mK111 * common1) +
                  (mK00 * mK10 * rAdab) + (mK01 * mK11 * rAdab) +
                  (mK02 * mK12 * rAdab);
    float mPP02 = (mK20 * common3) + (mK21 * common1) + (mK221 * common2) +
                  (mK00 * mK20 * rAdab) + (mK01 * mK21 * rAdab) +
                  (mK02 * mK22 * rAdab);
    float mPP03 = (-mP03 * mK001) + (mK30 * common3) + (mK31 * common1) +
                  (mK32 * common2) - (mK01 * mP13) - (mK02 * mP23) +
                  (mK00 * mK30 * rAdab) + (mK01 * mK31 * rAdab) +
                  (mK02 * mK32 * rAdab);
    float mPP04 = (-mP04 * mK001) + (mK40 * common3) + (mK41 * common1) +
                  (mK42 * common2) - (mK01 * mP14) - (mK02 * mP24) +
                  (mK00 * mK40 * rAdab) + (mK01 * mK41 * rAdab) +
                  (mK02 * mK42 * rAdab);
    float mPP05 = (-mP05 * mK001) + (mK50 * common3) + (mK51 * common1) +
                  (mK52 * common2) - (mK01 * mP15) - (mK02 * mP25) +
                  (mK00 * mK50 * rAdab) + (mK01 * mK51 * rAdab) +
                  (mK02 * mK52 * rAdab);
    float mPP11 = (mK10 * common4) + (mK12 * common5) +
                  ((mK10 * mK10) * rAdab) + ((mK11 * mK11) * rAdab) +
                  ((mK12 * mK12) * rAdab) + (mK111 * common6);
    float mPP12 = (mK20 * common4) + (mK21 * common6) + (mK221 * common5) +
                  (mK10 * mK20 * rAdab) + (mK11 * mK21 * rAdab) +
                  (mK12 * mK22 * rAdab);
    float mPP13 = (-mP13 * mK111) + (mK30 * common4) + (mK31 * common6) +
                  (mK32 * common5) - (mK10 * mP03) - (mK12 * mP23) +
                  (mK10 * mK30 * rAdab) + (mK11 * mK31 * rAdab) +
                  (mK12 * mK32 * rAdab);
    float mPP14 = (-mP14 * mK111) + (mK40 * common4) + (mK41 * common6) +
                  (mK42 * common5) - (mK10 * mP04) - (mK12 * mP24) +
                  (mK10 * mK40 * rAdab) + (mK11 * mK41 * rAdab) +
                  (mK12 * mK42 * rAdab);
    float mPP15 = (-mP15 * mK111) + (mK50 * common4) + (mK51 * common6) +
                  (mK52 * common5) - (mK10 * mP05) - (mK12 * mP25) +
                  (mK10 * mK50 * rAdab) + (mK11 * mK51 * rAdab) +
                  (mK12 * mK52 * rAdab);
    float mPP22 = (mK20 * common7) + (mK21 * common8) +
                  ((mK20 * mK20) * rAdab) + ((mK21 * mK21) * rAdab) +
                  ((mK22 * mK22) * rAdab) + (mK221 * common9);
    float mPP23 = (-mP23 * mK221) + (mK30 * common7) + (mK31 * common8) +
                  (mK32 * common9) - (mK20 * mP03) - (mK21 * mP13) +
                  (mK20 * mK30 * rAdab) + (mK21 * mK31 * rAdab) +
                  (mK22 * mK32 * rAdab);
    float mPP24 = (-mP24 * mK221) + (mK40 * common7) + (mK41 * common8) +
                  (mK42 * common9) - (mK20 * mP04) - (mK21 * mP14) +
                  (mK20 * mK40 * rAdab) + (mK21 * mK41 * rAdab) +
                  (mK22 * mK42 * rAdab);
    float mPP25 = (-mP25 * mK221) + (mK50 * common7) + (mK51 * common8) +
                  (mK52 * common9) - (mK20 * mP05) - (mK21 * mP15) +
                  (mK20 * mK50 * rAdab) + (mK21 * mK51 * rAdab) +
                  (mK22 * mK52 * rAdab);
    float mPP33 = mP33 + ((mK30 * mK30) * rAdab) + ((mK31 * mK31) * rAdab) +
                  ((mK32 * mK32) * rAdab) + (mK30 * commonA) +
                  (mK31 * commonB) + (mK32 * commonC) - (mK30 * mP03) -
                  (mK31 * mP13) - (mK32 * mP23);
    float mPP34 = mP34 + (mK40 * commonA) + (mK41 * commonB) +
                  (mK42 * commonC) - (mK30 * mP04) - (mK31 * mP14) -
                  (mK32 * mP24) + (mK30 * mK40 * rAdab) +
                  (mK31 * mK41 * rAdab) + (mK32 * mK42 * rAdab);
    float mPP35 = mP35 + (mK50 * commonA) + (mK51 * commonB) +
                  (mK52 * commonC) - (mK30 * mP05) - (mK31 * mP15) -
                  (mK32 * mP25) + (mK30 * mK50 * rAdab) +
                  (mK31 * mK51 * rAdab) + (mK32 * mK52 * rAdab);
    float mPP44 =
            mP44 + ((mK40 * mK40) * rAdab) + ((mK41 * mK41) * rAdab) +
            ((mK42 * mK42) * rAdab) +
            (mK40 * ((-mP04) + (mK40 * mP00) + (mK41 * mP01) + (mK42 * mP02))) +
            (mK41 * ((-mP14) + (mK40 * mP01) + (mK41 * mP11) + (mK42 * mP12))) +
            (mK42 * ((-mP24) + (mK40 * mP02) + (mK41 * mP12) + (mK42 * mP22))) -
            (mK40 * mP04) - (mK41 * mP14) - (mK42 * mP24);
    float mPP45 =
            mP45 +
            (mK50 * ((-mP04) + (mK40 * mP00) + (mK41 * mP01) + (mK42 * mP02))) +
            (mK51 * ((-mP14) + (mK40 * mP01) + (mK41 * mP11) + (mK42 * mP12))) +
            (mK52 * ((-mP24) + (mK40 * mP02) + (mK41 * mP12) + (mK42 * mP22))) -
            (mK40 * mP05) - (mK41 * mP15) - (mK42 * mP25) +
            (mK40 * mK50 * rAdab) + (mK41 * mK51 * rAdab) +
            (mK42 * mK52 * rAdab);
    float mPP55 =
            mP55 + ((mK50 * mK50) * rAdab) + ((mK51 * mK51) * rAdab) +
            ((mK52 * mK52) * rAdab) +
            (mK50 * (-mP05 + (mK50 * mP00) + (mK51 * mP01) + (mK52 * mP02))) +
            (mK51 * (-mP15 + (mK50 * mP01) + (mK51 * mP11) + (mK52 * mP12))) +
            (mK52 * (-mP25 + (mK50 * mP02) + (mK51 * mP12) + (mK52 * mP22))) -
            (mK50 * mP05) - (mK51 * mP15) - (mK52 * mP25);

    // Normalization of covariance
    float invLen = invSqrt((x0_ * x0_) + (x1_ * x1_) + (x2_ * x2_));
    float invLen3 = invLen * invLen * invLen;
    float invLen32 = (invLen3 * invLen3);

    float x1x1x2x2 = ((x1_ * x1_) + (x2_ * x2_));
    float x0x0x2x2 = ((x0_ * x0_) + (x2_ * x2_));
    float x0x0x1x1 = ((x0_ * x0_) + (x1_ * x1_));

    P00_ = invLen32 * (-x1x1x2x2 * (-mPP00 * x1x1x2x2 + mPP01 * x0_ * x1_ +
                                    mPP02 * x0_ * x2_) +
                       x0_ * x1_ *
                               (-mPP01 * x1x1x2x2 + mPP11 * x0_ * x1_ +
                                mPP12 * x0_ * x2_) +
                       x0_ * x2_ *
                               (-mPP02 * x1x1x2x2 + mPP12 * x0_ * x1_ +
                                mPP22 * x0_ * x2_));
    P01_ = invLen32 * (-x0x0x2x2 * (-mPP01 * x1x1x2x2 + mPP11 * x0_ * x1_ +
                                    mPP12 * x0_ * x2_) +
                       x0_ * x1_ *
                               (-mPP00 * x1x1x2x2 + mPP01 * x0_ * x1_ +
                                mPP02 * x0_ * x2_) +
                       x1_ * x2_ *
                               (-mPP02 * x1x1x2x2 + mPP12 * x0_ * x1_ +
                                mPP22 * x0_ * x2_));
    P02_ = invLen32 * (-x0x0x1x1 * (-mPP02 * x1x1x2x2 + mPP12 * x0_ * x1_ +
                                    mPP22 * x0_ * x2_) +
                       x0_ * x2_ *
                               (-mPP00 * x1x1x2x2 + mPP01 * x0_ * x1_ +
                                mPP02 * x0_ * x2_) +
                       x1_ * x2_ *
                               (-mPP01 * x1x1x2x2 + mPP11 * x0_ * x1_ +
                                mPP12 * x0_ * x2_));
    P03_ = -invLen3 *
           (-mPP03 * x1x1x2x2 + mPP13 * x0_ * x1_ + mPP23 * x0_ * x2_);
    P04_ = -invLen3 *
           (-mPP04 * x1x1x2x2 + mPP14 * x0_ * x1_ + mPP24 * x0_ * x2_);
    P05_ = -invLen3 *
           (-mPP05 * x1x1x2x2 + mPP15 * x0_ * x1_ + mPP25 * x0_ * x2_);
    P11_ = invLen32 * (-x0x0x2x2 * (-mPP11 * x0x0x2x2 + mPP01 * x0_ * x1_ +
                                    mPP12 * x1_ * x2_) +
                       x0_ * x1_ *
                               (-mPP01 * x0x0x2x2 + mPP00 * x0_ * x1_ +
                                mPP02 * x1_ * x2_) +
                       x1_ * x2_ *
                               (-mPP12 * x0x0x2x2 + mPP02 * x0_ * x1_ +
                                mPP22 * x1_ * x2_));
    P12_ = invLen32 * (-x0x0x1x1 * (-mPP12 * x0x0x2x2 + mPP02 * x0_ * x1_ +
                                    mPP22 * x1_ * x2_) +
                       x0_ * x2_ *
                               (-mPP01 * x0x0x2x2 + mPP00 * x0_ * x1_ +
                                mPP02 * x1_ * x2_) +
                       x1_ * x2_ *
                               (-mPP11 * x0x0x2x2 + mPP01 * x0_ * x1_ +
                                mPP12 * x1_ * x2_));
    P13_ = -invLen3 *
           (-mPP13 * x0x0x2x2 + mPP03 * x0_ * x1_ + mPP23 * x1_ * x2_);
    P14_ = -invLen3 *
           (-mPP14 * x0x0x2x2 + mPP04 * x0_ * x1_ + mPP24 * x1_ * x2_);
    P15_ = -invLen3 *
           (-mPP15 * x0x0x2x2 + mPP05 * x0_ * x1_ + mPP25 * x1_ * x2_);
    P22_ = invLen32 * (-x0x0x1x1 * (-mPP22 * x0x0x1x1 + mPP02 * x0_ * x2_ +
                                    mPP12 * x1_ * x2_) +
                       x0_ * x2_ *
                               (-mPP02 * x0x0x1x1 + mPP00 * x0_ * x2_ +
                                mPP01 * x1_ * x2_) +
                       x1_ * x2_ *
                               (-mPP12 * x0x0x1x1 + mPP01 * x0_ * x2_ +
                                mPP11 * x1_ * x2_));
    P23_ = -invLen3 *
           (-mPP23 * x0x0x1x1 + mPP03 * x0_ * x2_ + mPP13 * x1_ * x2_);
    P24_ = -invLen3 *
           (-mPP24 * x0x0x1x1 + mPP04 * x0_ * x2_ + mPP14 * x1_ * x2_);
    P25_ = -invLen3 *
           (-mPP25 * x0x0x1x1 + mPP05 * x0_ * x2_ + mPP15 * x1_ * x2_);
    P33_ = mPP33;
    P34_ = mPP34;
    P35_ = mPP35;
    P44_ = mPP44;
    P45_ = mPP45;
    P55_ = mPP55;

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
    x0_ = x0_ * invLen;
    x1_ = x1_ * invLen;
    x2_ = x2_ * invLen;

    float uNb0 = u0 - x3_;
    float uNb1 = u1 - x4_;
    float uNb2 = u2 - x5_;

    // calculate the second row (sr) from a rotated first row (rotation with bias
    // corrected gyroscope measurement)
    sr0_ = -fr1_ * x_last_[2] + fr2_ * x_last_[1] -
           dt_ * (x_last_[1] * (fr1_ * uNb0 - fr0_ * uNb1) +
                  x_last_[2] * (fr2_ * uNb0 - fr0_ * uNb2));
    sr1_ = fr0_ * x_last_[2] - fr2_ * x_last_[0] +
           dt_ * (x_last_[0] * (fr1_ * uNb0 - fr0_ * uNb1) -
                  x_last_[2] * (fr2_ * uNb1 - fr1_ * uNb2));
    sr2_ = -fr0_ * x_last_[1] + fr1_ * x_last_[0] +
           dt_ * (x_last_[0] * (fr2_ * uNb0 - fr0_ * uNb2) +
                  x_last_[1] * (fr2_ * uNb1 - fr1_ * uNb2));

    // save the estimated non-gravitational acceleration
    linear_a_[0] = (z0 - x0_) * g_;
    linear_a_[1] = (z1 - x1_) * g_;
    linear_a_[2] = (z2 - x2_) * g_;
}

void DcmAhrs::computeAngles()
{
    // alternative method estimating the whole rotation matrix
    // integrate full rotation matrix (using first row estimate in memory)
    // normalize the second row
    float invlen = invSqrt((sr0_ * sr0_) + (sr1_ * sr1_) + (sr2_ * sr2_));
    sr0_ *= invlen;
    sr1_ *= invlen;
    sr2_ *= invlen;

    // recompute the first row (ensure perpendicularity)
    fr0_ = sr1_ * x_last_[2] - sr2_ * x_last_[1];
    fr1_ = -sr0_ * x_last_[2] + sr2_ * x_last_[0];
    fr2_ = sr0_ * x_last_[1] - sr1_ * x_last_[0];

    // normalize the first row
    invlen = invSqrt((fr0_ * fr0_) + (fr1_ * fr1_) + (fr2_ * fr2_));
    fr0_ *= invlen;
    fr1_ *= invlen;
    fr2_ *= invlen;

    arm_atan2_f32(sr0_, fr0_, &edata_.yaw);
    arm_atan2_f32(x1_, x2_, &edata_.roll);
    edata_.pitch = asinf(x0_);
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
    _q[0] = x0_;
    _q[1] = x1_;
    _q[2] = x2_;
    _q[3] = x3_;
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

} // namespace IMU_DCM_AHRS
