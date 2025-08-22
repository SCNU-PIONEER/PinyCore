#include "Detector.hpp"

#include "dsp/fast_math_functions.h"

using namespace CHASSIS::BIPEDAL;

bool Detector::isFloating(float _fn) { return _fn < FN_THRESHOLD; }

bool Detector::isStop(float _expVel, float _curVel)
{
    return _expVel == 0.f && fabs(_curVel) < STOP_THRESHOLD;
}

bool Detector::detectTumble(const LegState_s &_legL, const LegState_s &_legR,
                            const INS_SYS::INSData_s &_ins)
{
    // TODO: maybe this is a weighted linear regression
    return false;
}

bool Detector::detectFreeze(const INS_SYS::INSData_s &_ins)
{
    if (fabs(_ins.body.gx) < DANG_FREEZE_THRESHOLD &&
        fabs(_ins.body.gy) < DANG_FREEZE_THRESHOLD &&
        fabs(_ins.body.gz) < DANG_FREEZE_THRESHOLD)
        freezeTime++;
    else
        freezeTime = 0;
    if (freezeTime > FREEZE_MAX_TIME) {
        freezeTime = 0;
        return true;
    } else {
        return false;
    }
}
