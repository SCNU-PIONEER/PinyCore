#pragma once
#include "../LKMotor.hpp"

namespace PINYMOTOR::LKMOTOR {

class MG4005 final : public LKMotor {
private:
    /**
     * @brief 检查基础配置
     */
    MotorTypeDef_e checkBaseConfig() override;

    /**
     * @brief 初始化MG4005电机模型参数
     */
    void initModelParams() override;

public:
    // MG4005硬件固定参数（不确定完全正确）
    static constexpr float TRQE_DATA_MAX = 2000.f;  // 扭矩最大编码值
    static constexpr float TRQE_MAX = 2.4f;         // 扭矩最大值
    static constexpr float CURR_DATA_MAX = 2048.f;  // 电流最大编码值
    static constexpr float CURR_MAX = 1.44f;        // 电流最大值
    static constexpr float TORQ_CONSTANT = 0.05f;   // 转矩常数
    static constexpr float REDUCTION_RATIO = 10.0f; // 减速比
    static constexpr float ENCODER_SPAN = 8192.f;   // 编码器范围
    static constexpr uint16_t TX_CURR_ID = 0x280;   // 电流控制发送ID
    static constexpr uint16_t RX_BASE_ID = 0x140;   // 接收基础ID

    /**
     * @brief 构造函数
     */
    MG4005(const char _name[16], InitConfig_s _config);
};

} // namespace PINYMOTOR::LKMOTOR