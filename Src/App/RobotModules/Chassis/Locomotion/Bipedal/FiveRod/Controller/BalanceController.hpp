#pragma once

#include "Matrix.hpp"

namespace CHASSIS::BIPEDAL {

union States_u {
    struct {
        float s, ds;
        float phi, dphi;
        float q, dq;
    };
    float _[6];
};

union Controls_u {
    struct {
        float tw;
        float tp;
    };
    float _[2];
};

class BalanceController {
public:
    BalanceController();

    void load(const States_u &_refX, const States_u &_curX);
    const Matrix<2, 1> &solve(float _l);

private:
    Matrix<6, 1> refX_;
    Matrix<6, 1> curX_;
    Matrix<2, 1> u_;
};

} // namespace CHASSIS::BIPEDAL
