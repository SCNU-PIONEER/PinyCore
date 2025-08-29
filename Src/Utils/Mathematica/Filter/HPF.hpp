#pragma once

#include <cmath>
#include <numbers>

namespace FILTER {

class HPF {
public:
    HPF(float _sampleRate, float _cutoffFrequency)
            : sampleRate_(_sampleRate), cutoffFrequency_(_cutoffFrequency)
    {
    }

    float process(float _input)
    {
        float output =
                (alpha_ * outputPrev_) + (alpha_ * (_input - inputPrev_));
        inputPrev_ = _input;
        return outputPrev_ = output;
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

    void reset() { outputPrev_ = inputPrev_ = 0.f; }

private:
    float alpha_;
    float sampleRate_;
    float cutoffFrequency_;
    float outputPrev_ = 0.f;
    float inputPrev_ = 0.f;

    void updateAlpha()
    {
        float rc = 1.f / (2.f * std::numbers::pi_v<float> * cutoffFrequency_);
        float dt = 1.f / sampleRate_;
        alpha_ = rc / (rc + dt);
        alpha_ = std::isnan(alpha_) ? 0.f : alpha_;
    }
};

} // namespace FILTER
