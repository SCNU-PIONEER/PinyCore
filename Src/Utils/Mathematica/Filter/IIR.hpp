#pragma once
#include <cstdint>
#include <cstring>
#include <numbers>
#include <unordered_map>
#include <cmath>


namespace FILTER {
enum class Ripple_e : uint8_t { NONE = 0, DB0_5 = 1, DB1 = 2, DB2 = 3, DB3 = 4 };

enum class FilterType_e : uint8_t { BUTTERWORTH = 1, CHEBYSHEV = 2 };

template <uint8_t RANK> struct LPFIIRCoeffs_s {
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

template <uint8_t RANK> struct BPFIIRCoeffs_s {
    float NUM[RANK + 1]{};
    float DEN[RANK + 1]{};
    float x[RANK]{};
    float y[RANK]{};
    float sampleRate_ = 0.f;
    float _lowCutoffFreq = 0.f;
    float _highCutoffFreq = 0.f;
    struct NormParam_s {
        float a[(RANK / 2) + 1]{};
        float epsilon_ = 0.f;
    } normParam_;
};

template <uint8_t RANK> class LPFIIR {
public:
    LPFIIR() = default;
    LPFIIR(float _sampleRate, float _cutoffFreq)
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
    LPFIIRCoeffs_s<RANK> coeffs_;

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

class LPFIIR2 : public LPFIIR<2> {
    const std::unordered_map<Ripple_e, LPFIIRCoeffs_s<2>::NormParam_s> IIR2PARAMS = {
        { Ripple_e::NONE, { .a = { 1.f, std::numbers::sqrt2, 1.f }, .epsilon_ = 0.f } },
        { Ripple_e::DB0_5, { .a = { 1.5162026f, 1.4256245f, 1.f }, .epsilon_ = 0.3493114f } },
        { Ripple_e::DB1, { .a = { 1.1025103f, 1.0977343f, 1.f }, .epsilon_ = 0.5088471f } },
        { Ripple_e::DB2, { .a = { 0.8230603f, 0.8038164f, 1.f }, .epsilon_ = 0.7647831f } },
        { Ripple_e::DB3, { .a = { 0.7079478f, 0.6448996f, 1.f }, .epsilon_ = 0.9976283f } }
    };

public:
    LPFIIR2() = delete;
    LPFIIR2(float _sampleRate, float _cutoffFreq, FilterType_e _type, Ripple_e _ripple)
            : LPFIIR<2>(_sampleRate, _cutoffFreq)
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

class LPFIIR3 : public LPFIIR<3> {
    const std::unordered_map<Ripple_e, LPFIIRCoeffs_s<3>::NormParam_s> IIR3PARAMS = {
        { Ripple_e::NONE, { .a = { 1.f, 2.f, 2.f, 1.f }, .epsilon_ = 0.f } },
        { Ripple_e::DB0_5, { .a = { 0.7156938f, 1.5348954f, 1.2529130f, 1.f }, .epsilon_ = 0.3493114f } },
        { Ripple_e::DB1, { .a = { 0.4913067f, 1.2384092f, 0.9883412f, 1.f }, .epsilon_ = 0.5088471f } },
        { Ripple_e::DB2, { .a = { 0.3268901f, 1.0221903f, 0.7378216f, 1.f }, .epsilon_ = 0.7647831f } },
        { Ripple_e::DB3, { .a = { 0.2505943f, 0.9283480f, 0.5972404f, 1.f }, .epsilon_ = 0.9976283f } }
    };

public:
    LPFIIR3() = delete;
    LPFIIR3(float _sampleRate, float _cutoffFreq, FilterType_e _type, Ripple_e _ripple)
            : LPFIIR<3>(_sampleRate, _cutoffFreq)
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

template <uint8_t RANK> class BPFIIR {
public:
    BPFIIR() = delete;
    BPFIIR(float _sampleRate, float _lowCutoffFreq, float _highCutoffFreq)
    {
        coeffs_.sampleRate_ = _sampleRate;
        coeffs_._lowCutoffFreq = _lowCutoffFreq;
        coeffs_._highCutoffFreq = _highCutoffFreq;
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
    BPFIIRCoeffs_s<RANK> coeffs_;

    void calcNormcoeffs()
    {
        float w2 = 2.f * std::numbers::pi_v<float> * coeffs_._highCutoffFreq / coeffs_.sampleRate_;
        float w1 = 2.f * std::numbers::pi_v<float> * coeffs_._lowCutoffFreq / coeffs_.sampleRate_;
        float D = 1.f / tanf((w2 - w1) / 2.f);
        float E = 2.f * cosf((w2 + w1) / 2.f) / cosf((w2 - w1) / 2.f);

        switch (RANK) {
        case 2: {
            float normCoeffs = coeffs_.normParam_.a[0] + (coeffs_.normParam_.a[1] * D);
            coeffs_.NUM[0] = 1.f / normCoeffs;
            coeffs_.NUM[1] = 0.f;
            coeffs_.NUM[2] = -coeffs_.NUM[0];

            coeffs_.DEN[0] = 1.f;
            coeffs_.DEN[1] = -(coeffs_.normParam_.a[1] * D * E) / normCoeffs;
            coeffs_.DEN[2] = (coeffs_.normParam_.a[1] * D - coeffs_.normParam_.a[0]) / normCoeffs;
            break;
        }
        case 4: {
            float normCoeffs =
                    coeffs_.normParam_.a[0] + (coeffs_.normParam_.a[1] * D) + (coeffs_.normParam_.a[2] * D * D);
            coeffs_.NUM[0] = 1.f / normCoeffs;
            coeffs_.NUM[1] = 0.f;
            coeffs_.NUM[2] = -2.f * coeffs_.NUM[0];
            coeffs_.NUM[3] = 0.f;
            coeffs_.NUM[4] = coeffs_.NUM[0];

            coeffs_.DEN[0] = 1.f;
            coeffs_.DEN[1] =
                    -((coeffs_.normParam_.a[1] * D * E) + (2.f * coeffs_.normParam_.a[2] * D * D * E)) / normCoeffs;
            coeffs_.DEN[2] = ((coeffs_.normParam_.a[2] * D * D * E * E) + (2.f * coeffs_.normParam_.a[2] * D * D) -
                              2 * coeffs_.normParam_.a[0]) /
                             normCoeffs;
            coeffs_.DEN[3] =
                    ((coeffs_.normParam_.a[1] * D * E) - (2.f * coeffs_.normParam_.a[2] * D * D * E)) / normCoeffs;
            coeffs_.DEN[4] =
                    (coeffs_.normParam_.a[0] - (coeffs_.normParam_.a[1] * D) + (coeffs_.normParam_.a[2] * D * D)) /
                    normCoeffs;
        }
        }
    };
};

class BPFIIR2 : public BPFIIR<2> {
    const std::unordered_map<Ripple_e, BPFIIRCoeffs_s<2>::NormParam_s> IIR1PARAMS = {
        { Ripple_e::NONE, { .a = { 1.f, 1.f }, .epsilon_ = 0.f } },
        { Ripple_e::DB0_5, { .a = { 2.8627752f, 1.f }, .epsilon_ = 0.3493114f } },
        { Ripple_e::DB1, { .a = { 1.9652267f, 1.f }, .epsilon_ = 0.5088471f } },
        { Ripple_e::DB2, { .a = { 1.3075603f, 1.f }, .epsilon_ = 0.7647831f } },
        { Ripple_e::DB3, { .a = { 1.0023773f, 1.f }, .epsilon_ = 0.9976283f } }
    };

public:
    BPFIIR2() = delete;
    BPFIIR2(float _sampleRate, float _lowCutoffFreq, float _highCutoffFreq, FilterType_e _type, Ripple_e _ripple)
            : BPFIIR<2>(_sampleRate, _lowCutoffFreq, _highCutoffFreq)
    {
        memcpy(&coeffs_.normParam_, &IIR1PARAMS.at(_ripple), sizeof(coeffs_.normParam_));
        calcNormcoeffs();
        if (_type == FilterType_e::CHEBYSHEV) {
            for (float &i : coeffs_.NUM)
                i /= coeffs_.normParam_.epsilon_;
        }
    }

private:
};

class BPFIIR4 : public BPFIIR<4> {
    const std::unordered_map<Ripple_e, BPFIIRCoeffs_s<4>::NormParam_s> IIR2PARAMS = {
        { Ripple_e::NONE, { .a = { 1.f, std::numbers::sqrt2, 1.f }, .epsilon_ = 0.f } },
        { Ripple_e::DB0_5, { .a = { 1.5162026f, 1.4256245f, 1.f }, .epsilon_ = 0.3493114f } },
        { Ripple_e::DB1, { .a = { 1.1025103f, 1.0977343f, 1.f }, .epsilon_ = 0.5088471f } },
        { Ripple_e::DB2, { .a = { 0.8230603f, 0.8038164f, 1.f }, .epsilon_ = 0.7647831f } },
        { Ripple_e::DB3, { .a = { 0.7079478f, 0.6448996f, 1.f }, .epsilon_ = 0.9976283f } }
    };

public:
    BPFIIR4() = delete;
    BPFIIR4(float _sampleRate, float _lowCutoffFreq, float _highCutoffFreq, FilterType_e _type, Ripple_e _ripple)
            : BPFIIR<4>(_sampleRate, _lowCutoffFreq, _highCutoffFreq)
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

template <uint8_t RANK> class BiquadIIRN {
public:
    BiquadIIRN(const float _den[(RANK + 1) / 2][3], const float _gain[(RANK + 1) / 2])
    {
        for (size_t i = 0; i < (RANK + 1) / 2; i++) {
            biquads_[i].NUM_[0] = 1.f;
            biquads_[i].NUM_[1] = (i == ((RANK + 1) / 2 - 1)) ? ((RANK % 2 == 0) ? 2.f : 1.f) : 2.f;
            biquads_[i].NUM_[2] = (i == ((RANK + 1) / 2 - 1)) ? ((RANK % 2 == 0) ? 1.f : 0.f) : 1.f;
            std::copy(_den[i], _den[i] + 3, biquads_[i].DEN_);
            gain_[i] = _gain[i];
        }
    }
    float process(float _in)
    {
        float out = _in;
        for (size_t i = 0; i < (RANK + 1) / 2; i++) {
            out = biquads_[i].process(out);
            out *= gain_[i];
        }
        return out;
    }

private:
    class BiquadIIR {
    public:
        BiquadIIR() = default;
        float process(float _in)
        {
            float out = _in;
            w[0] = out - DEN_[1] * w[1] - DEN_[2] * w[2];
            out = NUM_[0] * w[0] + NUM_[1] * w[1] + NUM_[2] * w[2];
            w[2] = w[1];
            w[1] = w[0];
            return out;
        }
        float w[3]{};
        float NUM_[3]{}, DEN_[3]{};
    };
    BiquadIIR biquads_[(RANK + 1) / 2];
    float gain_[(RANK + 1) / 2]{};
};

} // namespace FILTER
