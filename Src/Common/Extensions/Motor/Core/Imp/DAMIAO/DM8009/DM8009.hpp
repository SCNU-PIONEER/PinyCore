#pragma once
#include "../DMMotor.hpp"

static constexpr float DM8009_OMEGA_MAX = 125.9685f;
static constexpr float DM8009_P_MAX = 12.5f;
static constexpr float DM8009_V_MAX = 45.f;
static constexpr float DM8009_T_MAX = 54.f;
static constexpr float DM8009_MITKP_MAX = 1.f;
static constexpr float DM8009_MITKD_MAX = 4.f;
static constexpr float DM8009_CURR_TX_CODE_SPAN = 10000.f
;static constexpr float DM8009_CURR_MAX = 41.044777f; // DM上位机看电机上电打印反馈
static constexpr float DM8009_TORQ_MAX = 40.f;
// 关于转矩电流与转矩关系
// https://blog.csdn.net/sy243772901/article/details/82925212
static constexpr float DM8009_MAG_FLUX = 0.0044f;     // 磁通量
static constexpr float DM8009_NP = 21.f;        // 极对数
static constexpr float DM8009_INTER_RR = (9.f / 1.f); // 内部减速比
static constexpr float DM8009_TORQ_CONSTANT = 0.9745f;

namespace PINYMOTOR::DMMOTOR {

class DM8009 final : public DMMotor {
private:
    MotorTypeDef_e checkBaseConfig();
public:
    DM8009(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR::DMMOTOR
