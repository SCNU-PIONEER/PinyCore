#pragma once
#include "../DMMotor.hpp"

namespace PINYMOTOR::DMMOTOR {

class DM8009 final : public DMMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float OMEGA_MAX = 125.9685f;
    static constexpr float P_MAX = 12.57f;
    static constexpr float V_MAX = 10.47f; // 100rpm
    static constexpr float T_MAX = 40.f;
    static constexpr float MIT_KP_MAX = 1.f;
    static constexpr float MIT_KD_MAX = 4.f;
    static constexpr float CURR_TX_CODE_SPAN = 10000.f;
    static constexpr float CURR_RATED = 18.6f;
    static constexpr float TORQ_RATED = 20.f;
    static constexpr float CURR_MAX = 40.f; // DM上位机看电机上电打印反馈
    static constexpr float TORQ_MAX = T_MAX;
    // 关于转矩电流与转矩关系
    // https://blog.csdn.net/sy243772901/article/details/82925212
    static constexpr float MAG_FLUX = 0.0044f;           // 磁链 (每个电机因工艺而不同)
    static constexpr float NP = 21.f;                    // 极对数
    static constexpr float INTER_RR = (9.f / 1.f);       // 内部减速比
    static constexpr float KN = TORQ_RATED / CURR_RATED; // 近似常数

    DM8009(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);
};

} // namespace PINYMOTOR::DMMOTOR
