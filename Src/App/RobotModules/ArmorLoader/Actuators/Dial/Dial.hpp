#pragma once

#include "ArmorLoader.hpp"

#include "IMotor.hpp"

#include "IIR.hpp"

#include "MsgImpl.hpp"
#include "Referee.hpp"
#include "RxPacket.hpp"
#include "PacketImpl.hpp"
#include <cstdint>

namespace ARMORLOADER {

class ShooterCtrlRxPacket : public COMM::RxPacket<COMM::ArmorBoosterPacketType, 1> {
    friend class Dial;
    ShooterCtrlRxPacket() { registerCallback(); }
    void registerCallback() final;
};

class Dial : public ArmorLoader {
    union Motors_u {
        struct {
            PINYMOTOR::IMotor *dial;
        };
        PINYMOTOR::IMotor *_[1];
    };

public:
    static constexpr float OUTER_RR = 5.f / 2.f;

    static constexpr uint8_t BULLET_NUM_ROUND = 10;

    static constexpr float DIAL_MOTOR_FREQ = 500.f;

    static constexpr bool DIAL_REVERSE = false;

    static constexpr float BLOCK_CURRENT = 2.f;   // A
    static constexpr float BLOCK_SPEED_MIN = 1.f; // rad/s
    static constexpr uint16_t BLOCK_CNT_MAX = 200.f;
    static constexpr float BARREL_HEAT_THRESHOLD_OFFSET = 40.f;

    FSMState_e lastState = FSMState_e::STOP;

    Dial();

    PINYMOTOR::IMotor *dial() const;

    /**
     * @brief ctrl dial rotate in _torq
     * 
     * @param _torq 
     */
    void ctrl(float _torq);

    /**
     * @brief ctrl dial rotate in _vel
     * 
     * @param _velPID 
     * @param _vel rad/s
     */
    void ctrl(PID *_velPID, float _vel);

    /**
     * @brief ctrl dial rotate to _ang
     * 
     * @param _posPID 
     * @param _velPID 
     * @param _ang single circle ang in rad
     */
    void ctrlSingl(PID *_posPID, PID *_velPID, float _ang);

    /**
     * @brief ctrl dial rotate to _ang
     * 
     * @param _posPID 
     * @param _velPID 
     * @param _ang multiple circle ang in rad
     */
    void ctrlMulti(PID *_posPID, PID *_velPID, float _ang);

    void stop();
    void enter();
    void update(void *_param);

    bool isBlock() const;

private:
    Motors_u motors_;

    float torq_;

    FILTER::LPFIIR2 velFltr_ = { 500.f, 30.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE };

    ShooterCtrlRxPacket shooterCtrlRxPacket_;

    float barrelHeat_ = 0.f;
    uint16_t lastBarrelHeatFromReferee_ = 0.f;
    float vel_; // rad/s
    bool refereeFire_ = false;

    uint16_t blockCnt = 0.f;
};

}; // namespace ARMORLOADER
