#pragma once

#include "Matrix.hpp"

namespace CHASSIS::BIPEDAL {

class WBRController {
public:
    union States_u {
        struct {
            float s, ds;
            float yaw, dyaw;
            float phil, dphil;
            float phir, dphir;
            float q, dq;
        };
        float _[10];
    };

    union Controls_u {
        struct {
            float twl, twr;
            float tql, tqr;
        };
        float _[4];
    };

    WBRController();

    void load(const States_u &_refX, const States_u &_curX);
    const Matrix<4, 1> &solve(float _lL, float _lR, bool _isFloatingL,
                              bool _isFloatingR);

private:
    Matrix<10, 1> refX_;
    Matrix<10, 1> curX_;
    Matrix<4, 1> u_;
};

} // namespace CHASSIS::BIPEDAL
