#pragma once

#include <cmath>
#include <numbers>

class HPF {
public:
    HPF(float _sampleRate, float _cutoffFrequency)
            : sampleRate_(_sampleRate), cutoffFrequency_(_cutoffFrequency)
    {
    }

    float process(float _input)
    {
        float rc = 1.f / (2.f * std::numbers::pi_v<float> * cutoffFrequency_);
        float dt = 1.f / sampleRate_;
        float alpha = rc / (rc + dt);

        alpha = std::isnan(alpha) ? 0.f : alpha;

        float output = (alpha * outputPrev_) + (alpha * (_input - inputPrev_));

        inputPrev_ = _input;

        return outputPrev_ = output;
    }

    void setSampleRate(float _sampleRate) { sampleRate_ = _sampleRate; }

    void setCutoffFrequency(float _cutoffFrequency)
    {
        cutoffFrequency_ = _cutoffFrequency;
    }

    void reset() { outputPrev_ = inputPrev_ = 0.f; }

private:
    float sampleRate_;
    float cutoffFrequency_;
    float outputPrev_ = 0.f;
    float inputPrev_ = 0.f;
};