#include "WLController.hpp"

#include <cmath>

using namespace CHASSIS::BIPEDAL;

WLController::WLController() = default;

void WLController::load(const States_u &_refX, const States_u &_curX)
{
    for (uint8_t i = 0; i < 6; i++) {
        refX_[i][0] = _refX._[i];
        curX_[i][0] = _curX._[i];
    }
}

const Matrix<2, 1> &WLController::solve(float _l, bool _isFloating)
{
    const float &x = _l;
    float pow2f = x * x;
    float pow3f = pow2f * x;
    float arrayK[2 * 6] = { (843.4955407f * pow2f) - (269.4788639f * x) -
                                    (1007.291795f * pow3f) - (3.687061267f),
                            (277.525651f * pow2f) - (69.88385425f * x) -
                                    (389.7680173f * pow3f) + (2.84244622f),
                            (7.571534191f * pow2f) - (2.677102639f * x) -
                                    (9.457245351f * pow3f) - (0.9409495917f),
                            (13.85482809f * x) - (41.47436615f * pow2f) +
                                    (41.26194061f * pow3f) - (5.27071884f),
                            (560.8398125f * pow2f) - (177.3929519f * x) -
                                    (721.8527863f * pow3f) + (35.8700123f),
                            (89.39269194f * pow2f) - (27.4712717f * x) -
                                    (111.5631863f * pow3f) + (5.025149599f),
                            (556.9213846f * pow2f) - (173.5508102f * x) -
                                    (813.9321351f * pow3f) + (49.36271561f),
                            (3.263477121f * x) - (22.2082588f * pow2f) +
                                    (24.51743143f * pow3f) + (2.620688457f),
                            (1.735349317f * pow2f) - (0.1220640373f * x) -
                                    (10.99732267f * pow3f) + (0.9955718736f),
                            (41.17911744f * pow2f) - (14.79439485f * x) -
                                    (62.40723907f * pow3f) + (4.674864134f),
                            (195.186365f * x) - (545.2339442f * pow2f) +
                                    (591.0949377f * pow3f) + (21.08812019f),
                            (17.04418146f * x) - (48.94129366f * pow2f) +
                                    (56.14127059f * pow3f) + (1.603078041f) };

    Matrix<6, 1> err = refX_ - curX_;
    Matrix<2, 6> k(arrayK);

    if (_isFloating) {
        u_[0][0] = 0;
        u_[1][0] = err[0][0] * k[1][0] + err[0][1] * k[1][1];
    } else {
        u_ = k * err; // err = ref - cur
    }

    return u_;
}
