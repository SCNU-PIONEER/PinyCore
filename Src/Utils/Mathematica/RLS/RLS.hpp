#pragma once

#include <cstdint>
#include "Matrix.hpp"

template <int rank> 
class RLS
{
public:
    RLS(float _lamda) : lamda_(_lamda) 
    {
        this->init();
    }

    void init(){
        P_ = Matrix<rank,rank>::eye();
        K_ = Matrix<rank,rank>::zeros();
        inputVector_ = Matrix<rank,1>::zeros();
        estVector_ = Matrix<rank,1>::zeros();
        e_ = 0.0f;
    }

    void update(Matrix<rank,1> inputVector, float outPut_){
        inputVector_ = inputVector;
        e_ = outPut_ - (inputVector_.trans() * estVector_)[0][0];
        K_ = P_ * inputVector_ / ((inputVector_.trans() * P_ * inputVector_)[0][0] + lamda_);
        P_ = (P_ - K_ * inputVector_.trans() * P_) / lamda_;
        estVector_ += K_ * e_;
    }
    Matrix<rank, 1> &getEstVector_() const { return estVector_; }
    
private:
    Matrix<rank,rank> K_;
    Matrix<rank,rank> P_;
    Matrix<rank,1> inputVector_;
    Matrix<rank,1> estVector_;

    float lamda_;  // 0< lamda < 1
    float e_;
};


