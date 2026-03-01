#pragma once

#include "RLS.hpp"

template <int RANK> class Identification {
    static constexpr float DEFAULT_RLS_LAMDA = 0.999f;

public:
    Identification(float _lamda = DEFAULT_RLS_LAMDA) { rls_.setLamda(_lamda); }

protected:
    void regress(const Matrix<RANK, 1> &_inputVector, float _output) { rls_.update(_inputVector, _output); }
    const Matrix<RANK, 1> &getIdentifyVector() const { return rls_.getEstVector(); }

private:
    RLS<RANK> rls_{ DEFAULT_RLS_LAMDA };
};
