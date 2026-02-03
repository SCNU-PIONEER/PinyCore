#pragma once
#include <cstdint>
#include <cstring>
#include <numbers>
#include <unordered_map>
#include <cmath>


namespace FILTER {
enum class Ripple_e : uint8_t { NONE = 0, DB0_5 = 1, DB1 = 2, DB2 = 3, DB3 = 4 };

enum class FilterType_e : uint8_t { BUTTERWORTH = 1, CHEBYSHEV = 2 };

template <uint8_t RANK> struct IIRCoeffs_s {
    float NUM[RANK + 1]{};
    float DEN[RANK + 1]{};
    float x[RANK]{};
    float y[RANK]{};
    float sampleRate_ = 0.f;
    float cutoffFreq_ = 0.f;
    struct NormParam_s {
        float a[RANK + 1]{};
        float epsilon_ = 0.f;
    } normParam_;
};

template <uint8_t RANK> class IIR {
public:
    IIR() = default;
    IIR(float _sampleRate, float _cutoffFreq)
    {
        coeffs_.sampleRate_ = _sampleRate;
        coeffs_.cutoffFreq_ = _cutoffFreq;
    }
    float process(float _in)
    {
        float out = coeffs_.NUM[0] * _in;
        for (uint8_t i = 0; i < RANK; i++) {
            out += coeffs_.NUM[i + 1] * coeffs_.x[i] - coeffs_.DEN[i + 1] * coeffs_.y[i];
        }
        for (uint8_t i = RANK - 1; i > 0; i--) {
            coeffs_.x[i] = coeffs_.x[i - 1];
            coeffs_.y[i] = coeffs_.y[i - 1];
        }
        coeffs_.x[0] = _in;
        coeffs_.y[0] = out;
        return out;
    }

protected:
    IIRCoeffs_s<RANK> coeffs_;

    void calcNormcoeffs()
    {
        float w = tanf(std::numbers::pi_v<float> * coeffs_.cutoffFreq_ / coeffs_.sampleRate_);
        float w2 = w * w;
        float w3 = w2 * w;
        switch (RANK) {
        case 2: {
            float normCoeffs = (coeffs_.normParam_.a[0] * w2) + (coeffs_.normParam_.a[1] * w) + coeffs_.normParam_.a[2];
            coeffs_.NUM[0] = w2 / normCoeffs;
            coeffs_.NUM[1] = 2 * coeffs_.NUM[0];
            coeffs_.NUM[2] = coeffs_.NUM[0];
            coeffs_.DEN[0] = 1.f;
            coeffs_.DEN[1] = (2 * coeffs_.normParam_.a[0] * w2 - 2 * coeffs_.normParam_.a[2]) / normCoeffs;
            coeffs_.DEN[2] =
                    (coeffs_.normParam_.a[0] * w2 - coeffs_.normParam_.a[1] * w + coeffs_.normParam_.a[2]) / normCoeffs;
            break;
        }
        case 3: {
            float normCoeffs = (coeffs_.normParam_.a[0] * w3) + (coeffs_.normParam_.a[1] * w2) +
                               (coeffs_.normParam_.a[2] * w) + coeffs_.normParam_.a[3];
            coeffs_.NUM[0] = w3 / normCoeffs;
            coeffs_.NUM[1] = 3 * coeffs_.NUM[0];
            coeffs_.NUM[2] = 3 * coeffs_.NUM[0];
            coeffs_.NUM[3] = coeffs_.NUM[0];
            coeffs_.DEN[0] = 1.f;
            coeffs_.DEN[1] = (3 * coeffs_.normParam_.a[0] * w3 + coeffs_.normParam_.a[1] * w2 -
                              coeffs_.normParam_.a[2] * w - 3 * coeffs_.normParam_.a[3]) /
                             normCoeffs;
            coeffs_.DEN[2] = (3 * coeffs_.normParam_.a[0] * w3 - coeffs_.normParam_.a[1] * w2 -
                              coeffs_.normParam_.a[2] * w + 3 * coeffs_.normParam_.a[3]) /
                             normCoeffs;
            coeffs_.DEN[3] = (coeffs_.normParam_.a[0] * w3 - coeffs_.normParam_.a[1] * w2 +
                              coeffs_.normParam_.a[2] * w - coeffs_.normParam_.a[3]) /
                             normCoeffs;
            break;
        }
        default:
            break;
        }
    }
};

class IIR2 : public IIR<2> {
    const std::unordered_map<Ripple_e, IIRCoeffs_s<2>::NormParam_s> IIR2PARAMS = {
        { Ripple_e::NONE, { .a = { 1.f, std::numbers::sqrt2, 1.f }, .epsilon_ = 0.f } },
        { Ripple_e::DB0_5, { .a = { 1.5162026f, 1.4256245f, 1.f }, .epsilon_ = 0.3493114f } },
        { Ripple_e::DB1, { .a = { 1.1025103f, 1.0977343f, 1.f }, .epsilon_ = 0.5088471f } },
        { Ripple_e::DB2, { .a = { 0.8230603f, 0.8038164f, 1.f }, .epsilon_ = 0.7647831f } },
        { Ripple_e::DB3, { .a = { 0.7079478f, 0.6448996f, 1.f }, .epsilon_ = 0.9976283f } }
    };

public:
    IIR2() = default;
    IIR2(float _sampleRate, float _cutoffFreq, FilterType_e _type, Ripple_e _ripple) : IIR<2>(_sampleRate, _cutoffFreq)
    {
        memcpy(&coeffs_.normParam_, &IIR2PARAMS.at(_ripple), sizeof(coeffs_.normParam_));
        calcNormcoeffs();
        if (_type == FilterType_e::CHEBYSHEV) {
            for (float &i : coeffs_.NUM)
                i /= coeffs_.normParam_.epsilon_ * 2.f;
        }
    }

private:
};

class IIR3 : public IIR<3> {
    const std::unordered_map<Ripple_e, IIRCoeffs_s<3>::NormParam_s> IIR3PARAMS = {
        { Ripple_e::NONE, { .a = { 1.f, 2.f, 2.f, 1.f }, .epsilon_ = 0.f } },
        { Ripple_e::DB0_5, { .a = { 0.7156938f, 1.5348954f, 1.2529130f, 1.f }, .epsilon_ = 0.3493114f } },
        { Ripple_e::DB1, { .a = { 0.4913067f, 1.2384092f, 0.9883412f, 1.f }, .epsilon_ = 0.5088471f } },
        { Ripple_e::DB2, { .a = { 0.3268901f, 1.0221903f, 0.7378216f, 1.f }, .epsilon_ = 0.7647831f } },
        { Ripple_e::DB3, { .a = { 0.2505943f, 0.9283480f, 0.5972404f, 1.f }, .epsilon_ = 0.9976283f } }
    };

public:
    IIR3() = default;
    IIR3(float _sampleRate, float _cutoffFreq, FilterType_e _type, Ripple_e _ripple) : IIR<3>(_sampleRate, _cutoffFreq)
    {
        memcpy(&coeffs_.normParam_, &IIR3PARAMS.at(_ripple), sizeof(coeffs_.normParam_));
        calcNormcoeffs();
        if (_type == FilterType_e::CHEBYSHEV) {
            for (float &i : coeffs_.NUM)
                i /= coeffs_.normParam_.epsilon_ * 4.f;
        }
    }

private:
};

template <uint8_t RANK> class IIRN {
public:
    IIRN() = default;
    IIRN(const float _num[RANK + 1], const float _den[RANK + 1])
    {
        memcpy(NUM_, _num, sizeof(NUM_));
        memcpy(DEN_, _den, sizeof(DEN_));
    }
    float process(float _in)
    {
        float out = NUM_[0] * _in;
        for (uint8_t i = 0; i < RANK; i++) {
            out += NUM_[i + 1] * x_[i] - DEN_[i + 1] * y_[i];
        }
        for (uint8_t i = RANK - 1; i > 0; i--) {
            x_[i] = x_[i - 1];
            y_[i] = y_[i - 1];
        }
        x_[0] = _in;
        y_[0] = out;
        return out;
    }

private:
    float x_[RANK]{}, y_[RANK]{};
    float NUM_[RANK + 1]{}, DEN_[RANK + 1]{};
};
} // namespace FILTER
