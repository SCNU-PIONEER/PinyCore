#include "ArmOreBin.hpp"
#include "StmLog.hpp"
#include "Cmd.hpp"

using namespace ARM;

void OreBin::init()
{
    pwm1 = std::make_unique<Pwm>(&htim1, TIM_CHANNEL_1);
    pwm2 = std::make_unique<Pwm>(&htim1, TIM_CHANNEL_3);
    pwm1->start();
    pwm2->start();

    LOG::info("OreBin", "init");
}

void OreBin::loop(ArmMsg_s &_msg)
{
    if (_msg.isStucked) {
        pwm1->setDutyCycle(stuck_angle);
        pwm2->setDutyCycle(stuck_angle);
    } else if (!_msg.isStucked) {
        pwm1->setDutyCycle(release_angle);
        pwm2->setDutyCycle(release_angle);
    }
}

void OreBin::stop()
{
    LOG::info("OreBin", "stop");
    pwm1->stop();
    pwm2->stop();
}