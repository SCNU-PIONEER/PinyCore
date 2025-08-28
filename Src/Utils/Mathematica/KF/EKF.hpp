#pragma once

#include "KF.hpp"

#include <functional>

template <int stateDim, int ctrlDim, int measDim>
class EKF : public KF<stateDim, ctrlDim, measDim> {
public:
    using StateFunc = std::function<Matrix<stateDim, 1>(
            const Matrix<stateDim, 1> &, const Matrix<ctrlDim, 1> &)>;
    using StateJacFunc = std::function<Matrix<stateDim, stateDim>(
            const Matrix<stateDim, 1> &, const Matrix<ctrlDim, 1> &)>;

    using MeasFunc =
            std::function<Matrix<measDim, 1>(const Matrix<stateDim, 1> &)>;
    using MeasJacFunc = std::function<Matrix<measDim, stateDim>(
            const Matrix<stateDim, 1> &)>;

    EKF(StateFunc _stateFunc, StateJacFunc _stateJacFunc, MeasFunc _measFunc,
        MeasJacFunc _measJacFunc, const Matrix<stateDim, stateDim> &_sQ,
        const Matrix<measDim, measDim> &_sR)
            : KF<stateDim, ctrlDim, measDim>(Matrix<stateDim, stateDim>::eye(),
                                             Matrix<stateDim, ctrlDim>::zeros(),
                                             Matrix<stateDim, measDim>::zeros(),
                                             _sQ, _sR)
            , f(_stateFunc)
            , jacF(_stateJacFunc)
            , h(_measFunc)
            , jacH(_measJacFunc)
    {
    }

    EKF() = default;


    void process()
    {
        prioriEstimation();
        linearizeStateModel();
        this->predictionUpdate();
        linearizeMeasurementModel();
        this->measurementUpdate();
        fusion();
        this->correctedVariance();
    }

private:
    StateFunc f = nullptr;       // nonlinear state transition function
    StateJacFunc jacF = nullptr; // state transition Jacobian function
    MeasFunc h = nullptr;        // nonlinear measurement function
    MeasJacFunc jacH = nullptr;  // measurement Jacobian function

    void linearizeStateModel() { this->A = jacF(this->x_, this->u); }
    void linearizeMeasurementModel() { this->H = jacH(this->x_); }

    void prioriEstimation() { this->x_ = f(this->x_, this->u); }
    void fusion() { this->x_ = this->x_ + this->K_ * (this->z - h(this->x_)); }
};
