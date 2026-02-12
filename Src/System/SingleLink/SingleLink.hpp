#pragma once

#include "LPF.hpp"
#include "RLS.hpp"
#include "IIR.hpp"

class YawSI {
    static constexpr uint8_t RANK = 3;

public:
    YawSI(float _ctrlFreq, float _cutoffFreq)
            : ctrlFreq_(_ctrlFreq)
            , velFilter_(_ctrlFreq, _cutoffFreq, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE)
            , torqFilter_(_ctrlFreq, _cutoffFreq, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE)
            , inerFilter_(_ctrlFreq, 1.f)
    {
    }

    void update(float _torq, float _vel);

private:
    Matrix<RANK, 1> systemMatrix_{ Matrix<RANK, 1>::ones() * 0.0001f };
    RLS<RANK> rls_{ 0.999f };

    float ctrlFreq_ = 0.f;
    FILTER::IIR3 velFilter_;
    FILTER::IIR3 torqFilter_;
    FILTER::LPF inerFilter_;

    float fitTorq_ = 0.f;
    float inertia_ = 0.f;
    float friction_ = 0.f;
    float coulomb_ = 0.f;
    float lastTheta_ = 0.f;
    float lastVel_ = 0.f;
};
