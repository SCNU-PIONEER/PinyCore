#pragma once

#include "ArmorLoader.hpp"

#include "IMotor.hpp"

#include "IIR.hpp"

namespace ARMORLOADER {

class Dial : public ArmorLoader {
    union Motors_u {
        struct {
            PINYMOTOR::IMotor *dial;
        };
        PINYMOTOR::IMotor *_[1];
    };

public:
    static constexpr float OUTER_RR = 1.f;

    static constexpr uint8_t BULLET_NUM_ROUND = 8;

    static constexpr float DIAL_MOTOR_FREQ = 500.f;

    static constexpr bool DIAL_REVERSE = true;

    static constexpr float BLOCK_CURRENT = 2.5f;  // A
    static constexpr float BLOCK_SPEED_MIN = 1.f; // rad/s
    static constexpr uint16_t BLOCK_CNT_MAX = 1500.f;

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
     * @param _ang multiple circle ang in rad
     */
    void ctrl(PID *_posPID, PID *_velPID, float _ang);

    void stop();
    void enter();
    void update(void *_param);

    bool isBlock() const;

private:
    Motors_u motors_;

    ArmorLoaderCtrlMsg_s armorLoaderCtrlMsg_;
    Subscriber<ArmorLoaderCtrlMsg_s> *armorLoaderCtrlSub_;

    FILTER::IIR2 velFltr_;

    float vel_; // rad/s

    uint16_t blockCnt = 0.f;
};

}; // namespace ARMORLOADER
