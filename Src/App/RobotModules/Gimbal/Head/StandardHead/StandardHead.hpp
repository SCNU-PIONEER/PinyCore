#pragma once

#include "Head.hpp"

namespace GIMBAL {

class StandardHead : public Head {
    static constexpr uint8_t MOTOR_NUM = 2;
    static constexpr float F_PI = std::numbers::pi_v<float>;

public:
    StandardHead();

    void stop() final;
    void enter() final;
    void update() final;
    PINYMOTOR::IMotor *joint(uint8_t _jId) const final;

protected:
private:
    // Vison_s vsData = {};
    // Subscriber<Vison_s> *vsSub;

    PINYMOTOR::IMotor *motor_[MOTOR_NUM]; // TODO: MotorTopic
};

}; // namespace GIMBAL
