#include "Bipedal.hpp"

#include "MotorCommonMacros.hpp"

using namespace CHASSIS;
using namespace PINYMOTOR;

void Bipedal::bal(const WBRController::States_s &_ref)
{
    WBRController ::States_s errData;
    errData.s = _ref.s - s_;
    errData.ds = std::clamp(_ref.ds, -MAX_VEL_BACK, MAX_VEL_FRONT) - ds_;
    errData.yaw = getMinorArc(_ref.yaw, insData_.yaw);
    errData.dyaw = _ref.dyaw - insData_.body.gz;
    errData.phil = getMinorArc(_ref.phil + PHI_OFFSET, phiGlobalL());
    errData.dphil = _ref.dphil - (legL_.state().dphi0 + insData_.body.gy);
    errData.phir = getMinorArc(_ref.phir + PHI_OFFSET, phiGlobalR());
    errData.dphir = _ref.dphir - (legR_.state().dphi0 + insData_.body.gy);
    errData.q = getMinorArc(_ref.q + Q_OFFSET, insData_.pitch);
    errData.dq = _ref.dq - insData_.body.gy;
    Matrix<10, 1> err{ errData.data };
    Matrix<4, 1> u = WBRController::solve(err, legL_.state().L0, legR_.state().L0, legL_.floatFlag, legR_.floatFlag);
    legL_.setTw(u(0, 0));
    legR_.setTw(u(1, 0));
    legL_.setTp(u(2, 0) - antiSplitTp_);
    legR_.setTp(u(3, 0) + antiSplitTp_);
}

void Bipedal::bal(const WLController::States_s &_ref, float _yaw) { bal(_ref, _ref, _yaw); }

void Bipedal::bal(const WLController::States_s &_refL, const WLController::States_s &_refR, float _yaw)
{
    WLController::States_s errDataL;
    errDataL.phi = getMinorArc(_refL.phi + PHI_OFFSET, phiGlobalL());
    errDataL.dphi = _refL.dphi - (legL_.state().dphi0 + insData_.body.gy);
    errDataL.s = _refL.s - s_;
    errDataL.ds = std::clamp(_refL.ds, -MAX_VEL_BACK, MAX_VEL_FRONT) - ds_;
    errDataL.q = getMinorArc(_refL.q + Q_OFFSET, insData_.pitch);
    errDataL.dq = _refL.dq - insData_.body.gy;
    Matrix<6, 1> errL{ errDataL.data };
    WLController::States_s errDataR;
    errDataR.phi = getMinorArc(_refR.phi + PHI_OFFSET, phiGlobalR());
    errDataR.dphi = _refR.dphi - (legR_.state().dphi0 + insData_.body.gy);
    errDataR.s = _refR.s - s_;
    errDataR.ds = std::clamp(_refR.ds, -MAX_VEL_BACK, MAX_VEL_FRONT) - ds_;
    errDataR.q = getMinorArc(_refR.q + Q_OFFSET, insData_.pitch);
    errDataR.dq = _refR.dq - insData_.body.gy;
    Matrix<6, 1> errR{ errDataR.data };
    Matrix<2, 1> uL = WLController::solve(errL, legL_.state().L0, legL_.floatFlag);
    Matrix<2, 1> uR = WLController::solve(errR, legR_.state().L0, legR_.floatFlag);
    float tw2 = yawPid_[1].calc(yawPid_[0].calc(_yaw, insData_.yaw), wz_);
    legL_.setTw(uL(0, 0) + tw2);
    legL_.setTp(uL(1, 0) - antiSplitTp_);
    legR_.setTw(uR(0, 0) - tw2);
    legR_.setTp(uR(1, 0) + antiSplitTp_);
}

void Bipedal::bal(const STController::States_s &_ref, float _yaw)
{
    STController::States_s errData;
    errData.s = _ref.s - s_;
    errData.q = getMinorArc(_ref.q + Q_OFFSET, insData_.pitch);
    errData.ds = std::clamp(_ref.ds, -MAX_VEL_BACK, MAX_VEL_FRONT) - ds_;
    errData.dq = _ref.dq - insData_.body.gy;

    Matrix<4, 1> err{ errData.data };
    float lAvg = (legL_.state().L0 + legR_.state().L0) / 2.f;
    bool isf = isFloating();
    Matrix<1, 1> uDiv = STController::solve(err, lAvg, isf) / 2.f;
    float tw2 = yawPid_[1].calc(yawPid_[0].calc(_yaw, insData_.yaw), wz_);
    legL_.setTw((uDiv(0, 0) * W_RADIUS) + tw2);
    legR_.setTw((uDiv(0, 0) * W_RADIUS) - tw2);

    float hipVelL = hipAngPidL_.calc(getMinorArc(PHI_OFFSET, phiL()), 0);
    float tpL = hipVelPidL_.calc(hipVelL, legL_.state().dphi0);
    legL_.setTp(tpL - antiSplitTp_);
    float hipVelR = hipAngPidR_.calc(getMinorArc(PHI_OFFSET, phiR()), 0);
    float tpR = hipVelPidR_.calc(hipVelR, legR_.state().dphi0);
    legR_.setTp(tpR + antiSplitTp_);
}

void Bipedal::noBal(float _vx, float _wz)
{
    float twL = dsPid_[0].calc(_vx + (_wz * W_DISTANCE_HALF), motors_.WheelL->vel() * W_RADIUS);
    float twR = dsPid_[1].calc(_vx - (_wz * W_DISTANCE_HALF), motors_.WheelR->vel() * W_RADIUS);
    legL_.setTw(twL);
    legR_.setTw(twR);
}