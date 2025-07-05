#pragma once

#include "Pid.hpp"

typedef struct {
    float Kp;       /**< The proportional gain. */
    float Ki;       /**< The integral gain. */
    float Kd;       /**< The derivative gain. */
    float outMax;   /**< The maximum output value. */
    float deadband; /**< The deadband value. */
} incrementalPid_s;

typedef struct {
    float kp;       /**< The proportional gain. */
    float ki;       /**< The integral gain. */
    float kd;       /**< The derivative gain. */
    float dt;       /**< The time step for the PID calculation. */
    float iMax;     /**< The maximum integral output value. */
    float outMax;   /**< The maximum output value. */
    float deadband; /**< The deadband value. */
} positonalPid_s;

class incrementalPid : public PID {
public:
    /**
    * @brief  init incremental pid parameter
    */
    incrementalPid(incrementalPid_s &_pid);

    /** @brief  pid reset
    */
    void reset() override;

    /**
    * @brief  pid calculation
    */
    float calc(float ref, float cur) override;

private:
    float A0; /**< The derived gain, A0 = Kp + Ki + Kd . */
    float A1; /**< The derived gain, A1 = -Kp - 2Kd. */
    float A2; /**< The derived gain, A2 = Kd . */
    float state[3];
    incrementalPid_s &pid_;
};

class positonalPid : public PID {
public:
    /**
    * @brief  init positonal pid parameter
    */
    positonalPid(positonalPid_s &_pid);

    /** @brief  pid reset
    */
    void reset() override;

    /**
    * @brief  pid calculation
    */
    float calc(float ref, float cur) override;

private:
    positonalPid_s &pid_;
    float iOut;
    float err[2];
};
