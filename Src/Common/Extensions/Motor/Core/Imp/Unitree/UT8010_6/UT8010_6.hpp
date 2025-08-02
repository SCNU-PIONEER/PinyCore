#pragma once
#include "UTMotor.hpp"
#include <cstdint>

namespace PINYMOTOR::UTMOTOR {

class UT80106 : UTMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    /* 控制帧幅值 */
    static constexpr uint32_t P_MAX = 411774; // rad
    static constexpr float V_MAX = 804.0f;    // rad/s
    static constexpr float T_MAX = 127.99f;   // Nm
    static constexpr float KP_MAX = 25.599f;
    static constexpr float KD_MAX = 25.599f;
    /* 电机特性 */
    static constexpr uint32_t SPEED_MAX = 30; // rad/s
    static constexpr float TRQE_MAX = 23.7f;  // Nm
    static constexpr float CURR_MAX = 40.f;   // A
    static constexpr float KN = 0.63895f;     // Nm/A
    static constexpr float RR = 6.33f;        // 减速比

    UT80106(const char _name[16], PINYMOTOR::InitConfig_s _config,
            DMA_HandleTypeDef *_dmaHandle);
};

} // namespace PINYMOTOR::UTMOTOR
