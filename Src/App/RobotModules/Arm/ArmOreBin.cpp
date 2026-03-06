#include "ArmOreBin.hpp"
#include "StmLog.hpp"
#include "main.h"

using namespace ARM;

OreBin::OreBin() : leftOreBin(&htim1, TIM_CHANNEL_1), rightOreBin(&htim1, TIM_CHANNEL_3) {}

void OreBin::init()
{
    leftOreBin.start();
    rightOreBin.start();

    LOG::info("OreBin", "init");
}

void OreBin::loop(const ArmMsg_s &_msg)
{
    if (_msg.isStucked) {
        leftOreBin.setDutyCycle(L_STUCK_ANG);
        rightOreBin.setDutyCycle(R_STUCK_ANG);
    } else if (!_msg.isStucked) {
        leftOreBin.setDutyCycle(L_RELEASE_ANG);
        rightOreBin.setDutyCycle(R_RELEASE_ANG);
    }
}

void OreBin::stop()
{
    leftOreBin.stop();
    rightOreBin.stop();

    LOG::info("OreBin", "stop");
}
