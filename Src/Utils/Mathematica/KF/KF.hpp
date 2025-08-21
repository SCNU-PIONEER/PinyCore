#pragma once

#include "Matrix.hpp"

template <int stateDim, int measDim, int ctrlDim = 1> class KF {
    static constexpr float MIN_P_DIAG = 1e-5f;
    static constexpr float MAX_P_DIAG = 100.f;
    static constexpr float MAX_ABS_OFF_DIAG = 10.f;

public:
    KF(const Matrix<stateDim, stateDim> &_sF,
       const Matrix<stateDim, measDim> &_sH,
       const Matrix<stateDim, stateDim> &_sQ,
       const Matrix<measDim, measDim> &_sR)
            : F(_sF), H(_sH), Q(_sQ), R(_sR)
    {
    }

    KF(const Matrix<stateDim, stateDim> &_sF,
       const Matrix<stateDim, ctrlDim> &_sB,
       const Matrix<stateDim, measDim> &_sH,
       const Matrix<stateDim, stateDim> &_sQ,
       const Matrix<measDim, measDim> &_sR)
            : B(_sB), isCtrl(true)
    {
        KF(_sF, _sH, _sQ, _sR);
    }

    KF() = default;

    void process()
    {
        prioriEstimation();
        predictionUpdate();
        measurementUpdate();
        fusion();
        correctedVariance();
    }

    void reset()
    {
        x_ = Matrix<stateDim, 1>::zeros();
        P_ = Matrix<stateDim, stateDim>::eye();
    }

    void stabilizeP()
    {
        // forced symmetry
        P_ = (P_ + P_.trans()) * 0.5;

        // limit diagonal range
        for (size_t i = 0; i < stateDim; i++)
            P_[i][i] = std::fmax(std::fmin(P_[i][i], MAX_P_DIAG), MIN_P_DIAG);


        // restrict non diagonal elements
        for (int i = 0; i < P_.rows(); ++i) {
            for (int j = 0; j < i; ++j) {
                P_[i][j] = P_[j][i] = fmax(-MAX_ABS_OFF_DIAG,
                                           fmin(MAX_ABS_OFF_DIAG, P_[i][j]));
            }
        }
    }

    Matrix<ctrlDim, 1> u{ Matrix<ctrlDim, 1>::zeros() };
    Matrix<measDim, 1> z{ Matrix<measDim, 1>::zeros() };

    const float &x(int _i) const { return x_[0][_i]; }

    Matrix<stateDim, stateDim> F{
        Matrix<stateDim, stateDim>::zeros()
    }; // state transition matrix

    Matrix<stateDim, ctrlDim> B{
        Matrix<stateDim, ctrlDim>::zeros()
    }; // control matrix

    Matrix<stateDim, measDim> H{
        Matrix<stateDim, measDim>::zeros()
    }; // measurement matrix

    Matrix<stateDim, stateDim> Q{
        Matrix<stateDim, stateDim>::zeros()
    }; // process noise covariance

    Matrix<measDim, measDim> R{
        Matrix<stateDim, measDim>::zeros()
    }; // measurement noise covariance

protected:
    Matrix<stateDim, measDim> P_{ Matrix<stateDim, measDim>::eye() };
    Matrix<stateDim, measDim> K_{ Matrix<stateDim, measDim>::zeros() };

    Matrix<stateDim, 1> x_{ Matrix<stateDim, 1>::zeros() };

    bool isCtrl = false;

    void prioriEstimation()
    {
        // xhat'(k)= A·xhat(k-1) + B·u
        if (isCtrl)
            x_ = F * x_ + B * u;
        else
            x_ = F * x_;
    }

    void predictionUpdate()
    {
        // P'(k) = A·P(k-1)·A^T + Q
        P_ = F * P_ * F.trans() + Q;
    }

    void measurementUpdate()
    {
        // K(k) = P'(k)·H^T / (H·P'(k)·H^T + R)
        K_ = P_ * H.trans() * ((H * P_ * H.trans()) + R).inv();
    }

    void fusion()
    {
        // xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
        x_ = x_ + K_ * (z - H * x_);
    }

    void correctedVariance()
    {
        // P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
        P_ = P_ - K_ * H * P_;
    }
};
