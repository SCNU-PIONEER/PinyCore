#pragma once

#include "IMotor.hpp"
#include "Gimbal.hpp"

namespace GIMBAL {

enum class FSMState_e : uint8_t { STOP = 1, LAUNCH, RUN };

union Motors_u {
    struct {
        PINYMOTOR::IMotor *pitch;
        PINYMOTOR::IMotor *yaw;
    };
    PINYMOTOR::IMotor *_[2];
};

union States_u {
    struct {
        float pos;
        float vel;
    };
    float _[2];
};

class Standard : public Gimbal {
public:
    static constexpr float PITCH_LAUNCH_ANG = 2.08033204f;
    static constexpr float YAW_LAUNCH_ANG = 4.42377377f;

    static constexpr bool PITCH_REVERSE = false;
    static constexpr bool YAW_REVERSE = false;

    Standard();

    PINYMOTOR::IMotor *pitch() const;
    PINYMOTOR::IMotor *yaw() const;

    void stop();
    void enter();
    void update(void *_param);

    void updateEndYaw();
    void updateBaseYaw();

private:
    Motors_u motors_;
};

} // namespace GIMBAL
