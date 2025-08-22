#pragma once
#include "Locomotion.hpp"

#include "WheelLegged.hpp"

#include "WLController.hpp"
#include "WBRController.hpp"
#include "STController.hpp"

#include "./FiveRod/Detector/Detector.hpp"

namespace CHASSIS::BIPEDAL {

enum class Posture_e : uint8_t {
    NONE, // Balanced
    TUMBLE,
    SIT
};

class Bipedal {
    static constexpr float LF_HIP_ZERO_ANG = 4416.f;
    static constexpr float LB_HIP_ZERO_ANG = 14734.f;
    static constexpr float RF_HIP_ZERO_ANG = 8629.f;
    static constexpr float RB_HIP_ZERO_ANG = 12029.f;

    static constexpr float MAX_ABS_ROLL = 0.3f; // rad

public:
    Bipedal();

    void stop();
    void enter();
    void update();

    void setRoll(float _roll);

    void setL(float _l);
    void setL(float _lL, float _lR);

    void setFd(float _fd);
    void setFd(float _fdL, float _fdR);

    void ctrlBal(const WLController::States_u &_ref, float _wz);
    void ctrlBal(const WLController::States_u &_refL,
                 const WLController::States_u &_refR, float _wz);

    void ctrlBal(const WBRController::States_u &_ref);

    void ctrlBal(const STController::States_u &_ref);

    void ctrlNoBal(float _vx, float _wz);

    void ctrlHip(float _phi);
    void ctrlHip(float _phiL, float _phiR);

private:
    PINYMOTOR::IMotor *motor_[6];

    WLController balWLCtrlL_;
    WLController balWLCtrlR_;
    STController balSTCtrl_;
    WBRController balWBRCtrl_;

    WheelLegged legL_{ motor_[1], motor_[2], motor_[0] };
    WheelLegged legR_{ motor_[4], motor_[5], motor_[3] };

    INS_SYS::INSData_s insData_ = {};
    Subscriber<INS_SYS::INSData_s> *insSub_;

    Detector detector_;

    PositonalPid rollAngPid_{ 0, 0, 0, 0, 0, 0, 0 };
    PositonalPid rollVelPid_{ 0, 0, 0, 0, 0, 0, 0 };

    PositonalPid hipAngPidL_{ 0, 0, 0, 0, 0, 0, 0 };
    PositonalPid hipVelPidL_{ 0, 0, 0, 0, 0, 0, 0 };

    PositonalPid hipAngPidR_{ hipAngPidL_ };
    PositonalPid hipVelPidR_{ hipVelPidL_ };

    IncrementalPid vxPid_{ 0, 0, 0, 0, 0 };

    float setRoll_ = 0.f;

    float az_ = 0.f;
    float ax_ = 0.f;
};

} // namespace CHASSIS::BIPEDAL
