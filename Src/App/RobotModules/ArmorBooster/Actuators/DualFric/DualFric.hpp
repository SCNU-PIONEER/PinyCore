#pragma once

#include "ArmorBooster.hpp"

#include "IMotor.hpp"

#include "IIR.hpp"
#include "LPF.hpp"
#include "PacketImpl.hpp"
#include "PidBasic.hpp"
#include "RxPacket.hpp"
#include "Bsp_can.hpp"

namespace ARMORBOOSTER {

class SpeedRxPacket : public COMM::RxPacket<COMM::SpeedMsgPacket, 3> {
public:
    void registerCallback() final
    {
        Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
            BaseType_t higherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(this->queue_, _rxBuf, &higherPriorityTaskWoken);
        });
    }
};

class DualFric : public ArmorBooster {
    union Motors_u {
        struct {
            PINYMOTOR::IMotor *fricL;
            PINYMOTOR::IMotor *fricR;
        };
        PINYMOTOR::IMotor *_[2];
    };

public:
    static constexpr float FAST_FRIC_RATE = 618.f;
    static constexpr float SLOW_FRIC_RATE = 314.f;

    static constexpr float FRIC_STOP_THRESHOLD = 1.f;

    static constexpr float FRIC_MOTOR_FREQ = 1000.f;

    static constexpr bool FRIC_REVERSE = false;

    DualFric();

    void ctrl(float _vel);

    void on();
    void off();

    void stop();
    void enter();
    void update(void *_param);

private:
    Motors_u motors_;

    FILTER::LPFIIR3 velFltr_[2] = {
        { 500.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
        { 500.f, 20.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE },
    };

    PositionalPid fricPid_[2]{ { 0.0012f, 0.0001f, 1e-6f, 0.001f, 0.03f, 0.157f, 0.f },
                               { 0.0014f, 0.0001f, 1e-6f, 0.001f, 0.03f, 0.157f, 0.f } };

    // FILTER::LPF velFltr_[2] = {
    //     { 500.f, 20.f },
    //     { 500.f, 20.f },
    // };

    // PositionalPid fricPid_[2]{ { 0.0025f, 0.f, 8e-7f, 0.001f, 0.003f, 0.157f, 0.f },
    //                            { 0.0025f, 0.f, 8e-7f, 0.001f, 0.003f, 0.157f, 0.f } };

    SpeedRxPacket speedRxPacket_;

    float lErr_ = 0.f;
    float rErr_ = 0.f;

    float targetVel_ = SLOW_FRIC_RATE;
};

}; // namespace ARMORBOOSTER
