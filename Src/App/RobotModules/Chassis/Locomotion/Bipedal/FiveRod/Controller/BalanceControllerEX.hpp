#pragma once

#include "Matrix.hpp"

namespace CHASSIS::BIPEDAL {

union StatesEX_u {
    struct {
        float s, ds;
        float yaw, dyaw;
        float phil, dphil;
        float phir, dphir;
        float q, dq;
    };
    float _[10];
};

union ControlsEX_u {
    struct {
        float twl, twr;
        float tql, tqr;
    };
    float _[4];
};

class BalanceControllerEX {
public:
    BalanceControllerEX();

    void load(const StatesEX_u &_refX, const StatesEX_u &_curX);
    const Matrix<4, 1> &solve(StatesEX_u _err, float _ll, float _lr);

private:
    Matrix<10, 1> refX_;
    Matrix<10, 1> curX_;
    Matrix<4, 1> u_;
};

} // namespace CHASSIS::BIPEDAL
