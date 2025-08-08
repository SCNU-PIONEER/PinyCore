#pragma once

#include <cstdint>
#include "Matrix.hpp"

template <int rank> class RLS {
public:
    RLS(float _lamda)
            : lamda_(_lamda)
            , P_(Matrix<rank, rank>::eye() * 0.00001f)
            , K_(Matrix<rank, 1>::zeros())
            , inputVector_(Matrix<rank, 1>::zeros())
            , estVector_(Matrix<rank, 1>::zeros())
    {
    }
    void update(Matrix<rank, 1> _inputVector, float _outPut)
    {
        inputVector_ = _inputVector;
        e_ = _outPut - (inputVector_.trans() * estVector_)[0][0];
        K_ = (P_ * inputVector_) /
             ((inputVector_.trans() * P_ * inputVector_)[0][0] + lamda_);
        P_ = (P_ - K_ * inputVector_.trans() * P_) / lamda_;
        estVector_ += K_ * e_;
    }
    Matrix<rank, 1> const &getEstVector() { return estVector_; }

private:
    Matrix<rank, 1> K_;
    Matrix<rank, rank> P_;
    Matrix<rank, 1> inputVector_;
    Matrix<rank, 1> estVector_;

    float lamda_;    // 0< lamda < 1
    float e_ = 0.0f; // error
};
