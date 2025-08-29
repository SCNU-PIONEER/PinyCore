#pragma once

#include <cmath>
#include <numbers>

namespace FILTER {

/**
 * @brief 1-Euro filter implementation for adaptive signal smoothing
 * 
 * A simple speed-based low-pass filter for noisy input in interactive systems
 * Based on the paper: "1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input in Interactive Systems"
 */
class OneEuro {
public:
    OneEuro(float _minCutOff = 0.1f, float _beta = 0.05f, float _dCutOff = 2.f)
            : minCutOff_(_minCutOff), beta_(_beta), dCutOff_(_dCutOff)
    {
    }

    float process(float _in, float _sampleRate)
    {
        if (_sampleRate == 0.f)
            return _in;

        // Calculate and smooth the derivative
        float dx = (_in - xPrev_) * _sampleRate;
        float aD = 2.f * std::numbers::pi_v<float> * dCutOff_ / _sampleRate;
        aD = aD / (aD + 1.f);
        float dxHat = (aD * dx) + ((1.f - aD) * dxPrev_);

        // Adaptively adjust cutoff based on derivative
        float cutOff = minCutOff_ + (beta_ * std::fabs(dxHat));

        // Apply low-pass filter with adaptive cutoff
        aD = 2.f * std::numbers::pi_v<float> * cutOff / _sampleRate;
        aD = aD / (aD + 1.f);
        float xHat = (aD * _in) + ((1.f - aD) * xPrev_);

        xPrev_ = xHat;
        dxPrev_ = dxHat;

        return xHat;
    }

    void reset() { xPrev_ = dxPrev_ = 0.f; }

    void setParam(float _minCutOff, float _beta, float _dCutOff)
    {
        minCutOff_ = _minCutOff;
        beta_ = _beta;
        dCutOff_ = _dCutOff;
    }

private:
    float minCutOff_;
    float beta_;
    float dCutOff_;

    float xPrev_ = 0.f;
    float dxPrev_ = 0.f;
};

} // namespace FILTER
