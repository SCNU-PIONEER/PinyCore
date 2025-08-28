#pragma once

#include "Gimbal.hpp"

namespace GIMBAL::STANDARD {

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

class Standard : public Gimbal<Standard> {
public:
    static constexpr float PITCH_LAUNCH_ANG = 2.08033204f;
    static constexpr float YAW_LAUNCH_ANG = 4.42377377f;

    static constexpr bool PITCH_REVERSE = false;
    static constexpr bool YAW_REVERSE = false;

    Standard();

    PINYMOTOR::IMotor *pitch() const;
    PINYMOTOR::IMotor *yaw() const;

private:
    void stopSelf();
    void enterSelf();
    void updateSelf();

    void updateEndYaw();
    void updateBaseYaw();

    Motors_u motors_;

    friend class Gimbal<Standard>;
};

} // namespace GIMBAL::STANDARD
