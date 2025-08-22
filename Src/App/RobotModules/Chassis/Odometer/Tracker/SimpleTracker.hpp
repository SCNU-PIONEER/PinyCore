#pragma once

#include "KF.hpp"

namespace CHASSIS {

class SimpleTracker {
    static constexpr float DEFAULT_DT = 0.001f;

    struct WelFordData_s {
        uint16_t n = 0;
        float mean = 0.f, center = 0.f, variance = 0.f, val = 0.f;

        void reset() { *this = WelFordData_s{}; }
    };


public:
    SimpleTracker()
    {
        const float &dt = DEFAULT_DT;
        const float dt2 = dt * dt;
        const float dt3 = dt * dt2;
        CAKF.F = {
            { 1, DEFAULT_DT }, //
            { 0, 1 }           //
        };
        CAKF.H = Matrix<2, 2>::eye();
        CAKF.Q = {
            { dt3 * sigmaQ_ / 3.f, dt2 * sigmaQ_ / 2.f }, //
            { dt2 * sigmaQ_ / 2.f, dt * sigmaQ_ / 1.f }   //
        };
        CAKF.R = Matrix<2, 2>::eye() * sigmaR_;
    }

    void updateCA(float _vel, float _acc) { updateCA(DEFAULT_DT, _vel, _acc); }

    void updateCA(float _dt, float _vel, float _acc)
    {
        // update data variance
        updateVariance(accData_, _acc);
        updateVariance(velData_, _vel);

        // TODO: update sigma based on data variance

        // update matrix
        const float dt2 = _dt * _dt;
        const float dt3 = _dt * dt2;
        CAKF.F[0][1] = _dt;

        CAKF.Q[0][0] = dt3 * sigmaQ_ / 3.f;
        CAKF.Q[0][1] = dt2 * sigmaQ_ / 2.f;
        CAKF.Q[1][0] = dt2 * sigmaQ_ / 2.f;
        CAKF.Q[1][1] = _dt * sigmaQ_ / 1.f;

        CAKF.R[0][0] = sigmaR_;
        CAKF.R[1][1] = sigmaR_;

        CAKF.z[0][0] = _vel;
        CAKF.z[1][0] = _acc;

        // predict
        CAKF.process();
    }

    const float &obsVel() const { return CAKF.x(0); }
    const float &obsAcc() const { return CAKF.x(1); }

    KF<2, 2> CAKF; // constant acceleration model

private:
    void updateVariance(WelFordData_s &_data, float _val)
    {
        // Welch-Ford algorithm
        _data.val = _val;
        _data.n++;
        float delta = _val - _data.mean;
        _data.mean += delta / static_cast<float>(_data.n);
        _data.center = delta * (_val - _data.mean); // new mean
        _data.variance = _data.center /
                         static_cast<float>(_data.n - 1); // unbiased estimation
    }
    WelFordData_s accData_{};
    WelFordData_s velData_{};

    float sigmaQ_ = 10.f;
    float sigmaR_ = 1.f;
};

} // namespace CHASSIS
