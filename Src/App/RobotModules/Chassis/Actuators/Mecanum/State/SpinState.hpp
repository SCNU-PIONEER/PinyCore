#pragma once
#include "Mecanum.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

enum class SpinDir_e : uint8_t {
    LEFT = 0,
    RIGHT,
};

class SpinState : public FSMState<FSMState_e> {
    static constexpr float ACC = 0.5f;
    static constexpr float MAX_SPIN_SPEED = 5.0f;

public:
    SpinState(Mecanum &_chassis) : FSMState<FSMState_e>(FSMState_e::SPIN), chassis_(_chassis) {};

    void enter() final
    {
        chassis_.enter();
        LOG::info("ChassisRun", " enter");
    }

    void run() final
    {
        Speed_u ref{};
        ref.vx = std::fmax(chassis_.getSpeed().vx - ACC, std::fmin(chassis_.msg.vx, chassis_.getSpeed().vx + ACC));
        ref.vy = std::fmax(chassis_.getSpeed().vy - ACC, std::fmin(chassis_.msg.vy, chassis_.getSpeed().vy + ACC));
        ref.wz = getSpinVel(spinCnt_++);
        chassis_.ctrl(ref);
    }

    void exit() final
    {
        spinCnt_ = 0;
        spinDir_ = (spinDir_ == SpinDir_e::LEFT) ? SpinDir_e::RIGHT : SpinDir_e::LEFT;
        LOG::info("ChassisRun", " exit");
    }


    FSMState_e checkChange() final
    {
        if (chassis_.msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_.msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else
            return FSMState_e::RUN;
    }

    float getSpinVel(uint32_t _cnt)
    {
        float x = 2.1f * (static_cast<float>(_cnt) * 0.001f);
        float rslt = (MAX_SPIN_SPEED * sinf(x) * sinf(x)) + 2.5f;
        return static_cast<bool>(spinDir_) ? rslt : -rslt;
    }

private:
    Mecanum &chassis_;

    SpinDir_e spinDir_ = SpinDir_e::LEFT;
    uint32_t spinCnt_ = 0;
};
} // namespace CHASSIS
