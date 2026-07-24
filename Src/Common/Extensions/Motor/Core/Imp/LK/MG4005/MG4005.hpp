#pragma once
#include "../LKMotor.hpp"

namespace PINYMOTOR::LKMOTOR {

class MG4005 final : public LKMotor {
private:
    /**
     * @brief 检查基础配置
     */
    MotorTypeDef_e checkBaseConfig() final;

    /**
     * @brief 初始化MG4005电机模型参数
     */
    void initModelParams() final;

public:
    static constexpr float TX_CURR_DATA_MAX = 2000.f;   // 发送电流最大编码值
    static constexpr float RX_CURR_DATA_MAX = 4096.f;   // 接收电流最大编码值
    static constexpr float INNER_REDUCTION_RATIO = 1.f; // 内部减速比
    static constexpr float REDUCTION_RATIO = 10.0f;     // 减速比
    static constexpr float TRQE_MAX = 0.25f;            // 扭矩最大值
    static constexpr float TX_CURR_MAX = 4.1667f;       // 发送电流最大值
    static constexpr float RX_CURR_MAX = 66.f;          // 接收电流最大值
    static constexpr float TORQ_CONSTANT = 0.06f;       // 转矩常数
    static constexpr uint16_t ENCODER_SPAN = 65535.f;   // 编码器范围

    /**
     * @brief 构造函数
     */
    MG4005(const char _name[16], InitConfig_s _config, WorkMode_e _workmode);
};

} // namespace PINYMOTOR::LKMOTOR
