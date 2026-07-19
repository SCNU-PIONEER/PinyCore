#pragma once
#include "Smooth.hpp"
#include "Swerve.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

float spinAmendKp = 0.15f; // 旋转平移响应的等效延迟，单位：s

namespace CHASSIS {

class SpinState : public FSMState<FSMState_e> {
    static constexpr float ACC = 0.5f;
    static constexpr float MAX_AMEND_ANGLE = 0.35f;
    static constexpr float OFFSET_SPIN_SPEED = 0.5f;
    static constexpr float MAX_SPIN_SPEED = 2.f * 2.f;
    static constexpr float NORMAL_SPIN_SPEED = 0.5f * 2.f * PI; // 0.5圈/s

public:
    SpinState(Swerve *_chassis) : FSMState<FSMState_e>(FSMState_e::SPIN), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        spinCnt_ = 0;
        LOG::info("chassissSpin", "enter");
    }

    void run() final
    {
        Speed_u ref;
        ref.wz = MAX_SPIN_SPEED;

        //响应延迟补偿
        const float amendAngle = std::clamp(ref.wz * spinAmendKp, -MAX_AMEND_ANGLE, MAX_AMEND_ANGLE);
        const float cosAmend = cosf(amendAngle);
        const float sinAmend = sinf(amendAngle);
        const float amendVx = (cosAmend * chassis_->msg.vx) + (sinAmend * chassis_->msg.vy);
        const float amendVy = (-sinAmend * chassis_->msg.vx) + (cosAmend * chassis_->msg.vy);

        const float deltaVx = amendVx - chassis_->getSpeed().vx;
        const float deltaVy = amendVy - chassis_->getSpeed().vy;
        const float deltaSpeed = sqrtf((deltaVx * deltaVx) + (deltaVy * deltaVy));
        const float stepScale = deltaSpeed > ACC ? ACC / deltaSpeed : 1.0f; //变化太快还是会打滑
        ref.vx = chassis_->getSpeed().vx + (stepScale * deltaVx);
        ref.vy = chassis_->getSpeed().vy + (stepScale * deltaVy);

        // ref.wz = getSpinVel(spinCnt_++);
        // if (spinCnt_ + 5 >= MAX_SPIN_CNT)
        //     maxSpinCount = 0;
        // else if (spinCnt_ - 5 <= 0)
        //     maxSpinCount = MAX_SPIN_CNT;
        // SMOOTH::slopeFollowing(&maxSpinCount, &spinCnt_, 1.f);
        // ref.Speed.wz = MAX_SPIN_SPEED * (sin(omiga * spinCnt_) + 1.5f);
        chassis_->ctrl(ref);
    }

    void exit() final
    {
        chassis_->isGimbalReset = false;
        spinCnt_ = 0;
        spinDir_ = !spinDir_;
        LOG::info("ChassisSpin", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP || !chassis_->isGimbalReset)
            return FSMState_e::STOP;

        if (chassis_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (chassis_->msg.state == FSMState_e::TANK)
            return FSMState_e::TANK;
        else
            return FSMState_e::SPIN;
    }

    /// @brief speed change spinning
    float getSpinVel(uint32_t _cnt)
    {
        float x = 2.1f * (static_cast<float>(_cnt) * 0.001f);
        float rslt = (MAX_SPIN_SPEED * sinf(x) * sinf(x)) + 2.5f;
        return spinDir_ ? rslt : -rslt;
    }

private:
    Swerve *chassis_;
    bool spinDir_ = false; // true : left, false: right
    // uint32_t spinCnt_ = 0;
    static constexpr float SPIN_FREQ = 1000.f; //hz,代码运行频率
    static constexpr float PERIOD = 2;         //s
    static constexpr float MAX_SPIN_CNT = PERIOD * SPIN_FREQ;
    float spinCnt_ = 0;
    // float maxSpinCount = 0;
    // float omiga = 2 * PI / PERIOD / SPIN_FREQ; //rad/s ps:unused
};
} // namespace CHASSIS
