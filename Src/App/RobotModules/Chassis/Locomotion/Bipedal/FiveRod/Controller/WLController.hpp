#pragma once

#include "Matrix.hpp"

namespace CHASSIS::BIPEDAL {

class WLController {
public:
    union States_u {
        struct {
            float phi, dphi;
            float s, ds;
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

    WLController();

    void load(const States_u &_refX, const States_u &_curX);
    const Matrix<2, 1> &solve(float _l, bool _isFloating);

private:
    Matrix<6, 1> refX_;
    Matrix<6, 1> curX_;
    Matrix<2, 1> u_;
};

} // namespace CHASSIS::BIPEDAL
