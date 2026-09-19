#pragma once

#include "Identification.hpp"

#include "IIR.hpp"
#include "LPF.hpp"

class PitchSI : public Identification<4> {
    static constexpr uint8_t RANK = 4;

public:
    PitchSI(float _ctrlFreq, float _cutoffFreq, float _lamda = 0.999f)
            : Identification<RANK>(_lamda)
            , ctrlFreq_(_ctrlFreq)
            , velFilter_(_ctrlFreq, _cutoffFreq, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE)
            , torqFilter_(_ctrlFreq, _cutoffFreq, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE)
            , inerFilter_(_ctrlFreq, 1.f)
    {
    }

    void update(float _realTorq, float _realVel, float _realPos);

    float inertia() const { return inertia_; }
    float friction() const { return friction_; }
    float gravity() const { return gravity_; }
    float coulomb() const { return coulomb_; }

private:
    float ctrlFreq_ = 0.f;
    FILTER::LPFIIR3 velFilter_;
    FILTER::LPFIIR3 torqFilter_;
    FILTER::LPF inerFilter_;

    float fitTorq_ = 0.f;

    float inertia_ = 0.f;
    float friction_ = 0.f;
    float gravity_ = 0.f;
    float coulomb_ = 0.f;

    float lastTheta_ = 0.f;
    float lastVel_ = 0.f;
};