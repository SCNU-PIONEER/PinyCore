#pragma once

#include "Matrix.hpp"

// calc LQR gain online
template <int xRank, int uRank> class LQR {
    static constexpr float EPSILON = 1e-6;
    static constexpr unsigned short DEFAULT_MAX_ITER = 1000;

public:
    LQR(const Matrix<xRank, xRank> &_setA, const Matrix<xRank, uRank> &_setB,
        const Matrix<xRank, xRank> &_setQ, const Matrix<uRank, uRank> &_setR,
        unsigned short _maxIter = DEFAULT_MAX_ITER)
            : A_(_setA)
            , B_(_setB)
            , Q_(_setQ)
            , R_(_setR)
            , maxIter_(_maxIter)
            , P_(Q_)
    {
    }

    void setA(Matrix<xRank, xRank> &_setA) { A_ = _setA; }
    void setB(Matrix<xRank, uRank> &_setB) { B_ = _setB; }
    void setQ(Matrix<xRank, xRank> &_setQ) { Q_ = _setQ; }
    void setR(Matrix<uRank, uRank> &_setR) { R_ = _setR; }

    Matrix<uRank, xRank> &solveK()
    {
        // DP algorithm
        for (unsigned short iter = 0; iter < maxIter_; iter++) {
            // P(t) = Q + A'P(t+1)A - A'P(t+1)B(R+B'P(t+1)B)^(-1)B'P(t+1)A
            P_ = (Q_) + (A_.trans() * P_ * A_) -
                 (A_.trans() * P_ * B_ * (R_ + (B_.trans() * P_ * B_)).inv() *
                  B_.trans() * P_ * A_);
            if (P_.norm() < EPSILON)
                break;
        }
        // K = (R+B'P(t)B)^(-1)B'P(t)A
        K_ = (R_ + (B_.trans() * P_ * B_)).inv() * B_.trans() * P_ * A_;
        return K_;
    }

private:
    Matrix<xRank, xRank> A_;
    Matrix<xRank, uRank> B_;
    Matrix<xRank, xRank> Q_;
    Matrix<uRank, uRank> R_;

    Matrix<xRank, xRank> P_;

    Matrix<uRank, xRank> K_;

    unsigned short maxIter_;
};
