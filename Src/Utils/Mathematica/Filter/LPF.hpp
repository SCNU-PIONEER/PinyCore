#pragma once

#include <cmath>
#include <numbers>

class LPF {
public:
    LPF(float _sampleRate, float _cutoffFrequency)
            : sampleRate_(_sampleRate), cutoffFrequency_(_cutoffFrequency)
    {
    }

    float process(float _input)
    {
        float rc = 1.0f / (2.0f * std::numbers::pi_v<float> * cutoffFrequency_);
        float dt = 1.0f / sampleRate_;
        float alpha = dt / (rc + dt);

        alpha = std::isnan(alpha) ? 0.f : alpha;

        float output = (alpha * _input) + ((1.0f - alpha) * outputPrev_);

        return outputPrev_ = output;
    }

    void setSampleRate(float _sampleRate) { sampleRate_ = _sampleRate; }

    void setCutoffFrequency(float _cutoffFrequency)
    {
        cutoffFrequency_ = _cutoffFrequency;
    }

    void reset() { outputPrev_ = 0.f; }

private:
    float sampleRate_;
    float cutoffFrequency_;
    float outputPrev_ = 0.f;
};
