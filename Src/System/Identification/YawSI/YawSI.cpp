#include "YawSI.hpp"

void YawSI::update(float _realTorq, float _realVel)
{
    _realTorq = torqFilter_.process(_realTorq);
    _realVel = velFilter_.process(_realVel);
    float angAcc = (_realVel - lastVel_) / (1.0f / ctrlFreq_);
    lastVel_ = _realVel;

    Matrix<3, 1> varVector;
    varVector(0, 0) = angAcc;
    varVector(1, 0) = _realVel;
    varVector(2, 0) = (_realVel > 0.f) ? 1.f : ((_realVel < 0.f) ? -1.f : 0.f);

    if (std::isnan(varVector(0, 0)) || std::isnan(varVector(1, 0))) {
        return;
    }

    this->regress(varVector, _realTorq);
    const Matrix<3, 1> &param = this->getIdentifyVector();
    fitTorq_ = param(0, 0) * varVector(0, 0) + param(1, 0) * varVector(1, 0) + param(2, 0) * varVector(2, 0);
    inertia_ = inerFilter_.process(param(0, 0));
    friction_ = param(1, 0);
    coulomb_ = param(2, 0);
}