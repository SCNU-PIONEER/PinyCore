#include "WheelLegged.hpp"

#include "dsp/fast_math_functions.h"

using namespace CHASSIS::BIPEDAL;
using namespace PINYMOTOR;

void WheelLegged::init(PINYMOTOR::IMotor *_hip1, PINYMOTOR::IMotor *_hip4,
                       PINYMOTOR::IMotor *_wheel)
{
    hip1_ = _hip1;
    hip4_ = _hip4;
    wheel_ = _wheel;
}

void WheelLegged::stop()
{
    tp_ = fd_ = tw_ = 0.f;
    lengthAngPid_.reset();
    lengthVelPid_.reset();
    hip1_->cmd(MotorCmdType_e::OFF);
    hip4_->cmd(MotorCmdType_e::OFF);
    wheel_->cmd(MotorCmdType_e::OFF);
}

void WheelLegged::enter()
{
    hip1_->cmd(MotorCmdType_e::ON);
    hip4_->cmd(MotorCmdType_e::ON);
    wheel_->cmd(MotorCmdType_e::ON);
}

void WheelLegged::stateUpdate(float _dt)
{
    // TODO: maybe need a filter to smooth data
    legState_.phi1 = hip1_->posNorm();
    legState_.phi4 = hip4_->posNorm();

    legState_.dphi1 = hip1_->vel();
    legState_.dphi4 = hip4_->vel();

    legState_.tau1 = hip1_->torq();
    legState_.tau4 = hip4_->torq();

    vmc_.update(legState_, _dt);
}

float WheelLegged::calcFn(float _az)
{
    float cosPhi0 = arm_cos_f32(legState_.phi0);
    float sinPhi0 = arm_sin_f32(legState_.phi0);
    float fp =
            (legState_.fd * cosPhi0) + (legState_.tp * -sinPhi0 / legState_.L0);
    float azWheel = _az - (legState_.ad * cosPhi0) +
                    (2.f * legState_.vd * -legState_.dphi0 * -sinPhi0) +
                    (legState_.L0 * -legState_.ap * -sinPhi0) +
                    (legState_.L0 * powf(legState_.dphi0, 2) * cosPhi0);
    float fn = ((azWheel + GRAVITY) * MASS_WHEEL) + fp;
    return fn_ = fnLPF_.process(fn);
}

void WheelLegged::ctrl()
{
    float setVd = lengthAngPid_.calc(setL_, legState_.L0);
    fd_ = lengthVelPid_.calc(setVd, legState_.vd);

    Matrix<2, 1> u = vmc_.forwardTorq(tp_, fd_);
    hip1_->cmdTorq(u[0][0]);
    hip4_->cmdTorq(u[1][0]);
    wheel_->cmdTorq(tw_);
}

float WheelLegged::minL(float _phi)
{
    // TODO: a fitted polynomial : minL = f(phi)
    return 0.2f;
}

void WheelLegged::setL(float _l)
{
    setL_ = fmin(fmax(_l, minL(legState_.phi0)), MAX_L);
}


void WheelLegged::setFd(float _fd) { fd_ = _fd; }
void WheelLegged::setTp(float _tp) { tp_ = _tp; }
void WheelLegged::setTw(float _tw) { tw_ = _tw; }
