#pragma once

#include "Matrix.hpp"

namespace CHASSIS::BIPEDAL {

class STController {
public:
    union States_u {
        struct {
            float s, ds;
            float q, dq;
        };
        float _[4];
    };

    union Controls_u {
        struct {
            float tw;
        };
        float _[1];
    };

    STController();

    void load(const States_u &_refX, const States_u &_curX);
    const Matrix<1, 1> &solve(float _l, bool _isFloating);

private:
    Matrix<4, 1> refX_;
    Matrix<4, 1> curX_;
    Matrix<1, 1> u_;
};

} // namespace CHASSIS::BIPEDAL
