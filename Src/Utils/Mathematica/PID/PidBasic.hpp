#pragma once

#include "Pid.hpp"

class IncrementalPid : public PID {
public:
    /**
    * @brief  init incremental pid parameter
    */
    IncrementalPid(float _kp, float _ki, float _kd, float _outMax,
                   float _deadband);

    /** @brief  pid reset
    */
    void reset() override;

    /**
    * @brief  pid calculation
    */
    float calc(float _ref, float _cur) override;

protected:
    float A0; /**< The derived gain, A0 = Kp + Ki + Kd . */
    float A1; /**< The derived gain, A1 = -Kp - 2Kd. */
    float A2; /**< The derived gain, A2 = Kd . */
    float state[3];

private:
    float kp;       /**< The proportional gain. */
    float ki;       /**< The integral gain. */
    float kd;       /**< The derivative gain. */
    float outMax;   /**< The maximum output value. */
    float deadband; /**< The deadband value. */
};

class PositionalPid : public PID {
public:
    /**
    * @brief  init positonal pid parameter
    */
    PositionalPid(float _kp, float _ki, float _kd, float _dt, float _iMax,
                  float _outMax, float _deadband);

    /** @brief  pid reset
    */
    void reset() override;

    /**
    * @brief  pid calculation
    */
    float calc(float _ref, float _cur) override;

protected:
    float iOut;
    float err[2];

private:
    float kp;       /**< The proportional gain. */
    float ki;       /**< The integral gain. */
    float kd;       /**< The derivative gain. */
    float dt;       /**< The time step for the PID calculation. */
    float iMax;     /**< The maximum integral output value. */
    float outMax;   /**< The maximum output value. */
    float deadband; /**< The deadband value. */
};
