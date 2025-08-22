#include "STController.hpp"

#include <cmath>

using namespace CHASSIS::BIPEDAL;

STController::STController() = default;

void STController::load(const States_u &_refX, const States_u &_curX)
{
    for (uint8_t i = 0; i < 4; i++) {
        refX_[i][0] = _refX._[i];
        curX_[i][0] = _curX._[i];
    }
}

const Matrix<1, 1> &STController::solve(float _l, bool _isFloating)
{
    const float &x = _l;
    float pow2f = x * x;
    float pow3f = pow2f * x;
    float arrayK[1 * 4] = { (68.51118986f * pow2f) - (24.80887033f * x) -
                                    (67.11049178f * pow3f) - (27.15037162f),
                            (294.8148741f * x) - (302.9160592f * pow2f) +
                                    (260.8848726f * pow3f) + (178.7022458f),
                            (92.52413445f * pow2f) - (47.4874517f * x) -
                                    (86.25892003f * pow3f) - (33.86034365f),
                            (76.44522608f * x) + (2.787947509f * pow2f) +
                                    (8.817534524f * pow3f) + (8.255214623f) };
    Matrix<4, 1> err = refX_ - curX_;
    Matrix<1, 4> k(arrayK);
    if (_isFloating)
        u_ = k * err;
    else
        u_[0][0] = 0;
    return u_;
}