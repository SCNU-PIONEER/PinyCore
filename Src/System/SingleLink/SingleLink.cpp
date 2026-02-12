#include "SingleLink.hpp"

void YawSI::update(float _torq, float _vel)
{
    _torq = torqFilter_.process(_torq);
    _vel = velFilter_.process(_vel);
    float angAcc = (_vel - lastVel_) / (1.0f / ctrlFreq_);
    lastVel_ = _vel;

    Matrix<RANK, 1> inputVector;
    inputVector(0, 0) = angAcc;
    inputVector(1, 0) = _vel;
    inputVector(2, 0) = (_vel > 0.f) ? 1.f : ((_vel < 0.f) ? -1.f : 0.f);

    if (std::isnan(inputVector(0, 0)) || std::isnan(inputVector(1, 0))) {
        return;
    }
    rls_.update(inputVector, _torq);
    systemMatrix_ = rls_.getEstVector();
    fitTorq_ = 0.f;
    for (uint8_t i = 0; i < RANK; i++) {
        fitTorq_ += systemMatrix_(i, 0) * inputVector(i, 0);
    }
    inertia_ = inerFilter_.process(systemMatrix_(0, 0));
    friction_ = systemMatrix_(1, 0);
    coulomb_ = systemMatrix_(2, 0);
}
