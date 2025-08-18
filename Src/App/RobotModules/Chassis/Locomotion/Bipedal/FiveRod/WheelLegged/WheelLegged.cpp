#include "WheelLegged.hpp"

using namespace CHASSIS::BIPEDAL;
using namespace PINYMOTOR;

WheelLegged::WheelLegged(PINYMOTOR::IMotor *_hip1, PINYMOTOR::IMotor *_hip4,
                         PINYMOTOR::IMotor *_wheel)
        : hip1_(_hip1), hip4_(_hip4), wheel_(_wheel)
{
}

void WheelLegged::update()
{
    // TODO: maybe need a filter to smooth data
    legState_.phi1 = hip1_->posNorm();
    legState_.phi4 = hip4_->posNorm();

    legState_.dphi1 = hip1_->vel();
    legState_.dphi4 = hip4_->vel();

    legState_.tau1 = hip1_->torq();
    legState_.tau4 = hip4_->torq();

    vmc_.update(legState_);
}

void WheelLegged::ctrl()
{
    Matrix<2, 1> u = vmc_.forwardTorq(tp_, fd_);
    hip1_->cmdTorq(u[0][0]);
    hip4_->cmdTorq(u[1][0]);
    wheel_->cmdTorq(tw_);
}
