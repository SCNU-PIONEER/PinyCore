#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class SpinState : public FSMState {
public:
    SpinState(Chassis *_chassis) : chassis_(_chassis)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::SPIN));
    }

    void enter() final
    {
        //TODO: check the motor is offline
        chassis_->wheel->enter();
        LOG::info("ChassisSpin", " enter");
    }

    void run() final
    {
        chassis_->wheel->update();

        chassis_->convertCoordinate();

        if (chassis_->msg.state == State_e::run) {
            Speed_u refState{ .v_x = chassis_->msg.vx,
                              .v_y = chassis_->msg.vy,
                              .w_z = getSpinningSpeed() };
            chassis_->wheel->ctrl(refState);
        }
    }

    void exit() final { LOG::info("ChassisSpin", " exit"); };

    uint8_t checkChange() final
    {
        if (chassis_->msg.state == State_e::stop)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (chassis_->msg.state == State_e::run)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
    }

    /**
     * @brief 获取小陀螺转速
     * @return float 陀螺转速
     */
    float getSpinningSpeed()
    {
#if SPIN_DEBUG
        return Chassis::MAX_WZ_SPEED *
               (1 - sqrtf(powf(chassis_->msg.vy / Chassis::MAX_VY_SPEED, 2) +
                          powf(chassis_->msg.vx / Chassis::MAX_VX_SPEED, 2)) *
                            SPINNING_REDUCE_K);
#else
        static uint16_t spinAlgCnt = 1;
        spinAlgCnt = spinAlgCnt > 65534 ? 1 : spinAlgCnt + 1;
        return (sinf(static_cast<float>(log(spinAlgCnt))) * SPINNING_CYCLE_K +
                sqrtf(chassis_->wheel->getMaxPowerLimit()) * SPINNING_BASE_K -
                sqrtf(powf(chassis_->msg.vy, 2) + powf(chassis_->msg.vx, 2)) *
                        SPINNING_REDUCE_K) *
               0.10467f * 0.24f;
#endif
    }

private:
    /**
     * @brief 当前方案的速度函数为周期函数,振幅增益SPINNING_CYCLE_K,小陀螺增益SPINNING_BASE_K,
     *        底盘移动期望增益SPINNING_REDUCE_K
     */
    static constexpr float SPINNING_TEST_K = 42;
    static constexpr float SPINNING_CYCLE_K = 4.5f;
    static constexpr float SPINNING_BASE_K = 1.0f;
    static constexpr float SPINNING_REDUCE_K = 0.3f;

    Chassis *chassis_;
};

} //namespace CHASSIS