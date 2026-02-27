#pragma once
#include "Smooth.hpp"
#include "Swerve.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class SpinState : public FSMState<FSMState_e> {
    static constexpr float ACC = 0.5f;
    static constexpr float MAX_SPIN_SPEED = 5.0f;
    static constexpr float NORMAL_SPIN_SPEED = 0.4f * 2.f * PI;

public:
    SpinState(Swerve *_chassis) : FSMState<FSMState_e>(FSMState_e::SPIN), chassis_(_chassis) {}

    void enter() final
    {
        chassis_->enter();
        spinCnt_ = 0;
        LOG::info("ChassisSpin", " enter");
    }

    void run() final
    {
        Speed_u ref;
        ref.Speed.vx = std::fmax(chassis_->getSpeed().Speed.vx - ACC,
                                 std::fmin(chassis_->msg.vx, chassis_->getSpeed().Speed.vx + ACC));

        ref.Speed.vy = std::fmax(chassis_->getSpeed().Speed.vy - ACC,
                                 std::fmin(chassis_->msg.vy, chassis_->getSpeed().Speed.vy + ACC));
        // ref.wz = getSpinVel(spinCnt_++);
        if (spinCnt_ + 5 >= MAX_SPIN_CNT)
            maxSpinCount = 0;
        else if (spinCnt_ - 5 <= 0)
            maxSpinCount = MAX_SPIN_CNT;
        SMOOTH::slopeFollowing(&maxSpinCount, &spinCnt_, 1.f);
        ref.Speed.wz = NORMAL_SPIN_SPEED * (sin(omiga * spinCnt_) + 1);
        chassis_->ctrl(ref);
    }

    void exit() final
    {
        spinCnt_ = 0;
        spinDir_ = !spinDir_;
        LOG::info("ChassisSpin", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::RUN)
            return FSMState_e::RUN;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else if (chassis_->msg.state == FSMState_e::TANK)
            return FSMState_e::TANK;
        else
            return FSMState_e::STOP;
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
    static constexpr float PERIOD = 4;         //s
    static constexpr float MAX_SPIN_CNT = PERIOD * SPIN_FREQ;
    float spinCnt_ = 0;
    float maxSpinCount = 0;
    float omiga = 2 * PI / PERIOD / SPIN_FREQ; //rad/s
};
} // namespace CHASSIS
