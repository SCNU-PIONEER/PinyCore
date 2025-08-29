#pragma once

#include <cmath>
#include <numbers>

namespace FILTER {

class LPF {
public:
    LPF(float _sampleRate, float _cutoffFrequency)
            : sampleRate_(_sampleRate), cutoffFrequency_(_cutoffFrequency)
    {
    }

    float process(float _input)
    {
        return outputPrev_ =
                       (alpha_ * _input) + ((1.0f - alpha_) * outputPrev_);
    }

    void setSampleRate(float _sampleRate)
    {
        sampleRate_ = _sampleRate;
        updateAlpha();
    }

    void setCutoffFrequency(float _cutoffFrequency)
    {
        cutoffFrequency_ = _cutoffFrequency;
        updateAlpha();
    }

    void reset() { outputPrev_ = 0.f; }

private:
    float alpha_;
    float sampleRate_;
    float cutoffFrequency_;
    float outputPrev_ = 0.f;

    void updateAlpha()
    {
        float rc = 1.0f / (2.0f * std::numbers::pi_v<float> * cutoffFrequency_);
        float dt = 1.0f / sampleRate_;
        alpha_ = dt / (rc + dt);
        alpha_ = std::isnan(alpha_) ? 0.f : alpha_;
    }
};

} // namespace FILTER
