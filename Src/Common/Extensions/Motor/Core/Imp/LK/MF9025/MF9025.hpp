#pragma once
#include "LKMotor.hpp"

namespace PINYMOTOR::LKMOTOR {

class MF9025 final : public LKMotor {
private:
    /**
     * @brief 检查基础配置是否符合MF9025电机要求
     */
    MotorTypeDef_e checkBaseConfig() override;
    
    /**
     * @brief 初始化MF9025电机模型参数
     */
    void initModelParams() override;

public:
    // MF9025电机常量参数
    static constexpr float TRQE_DATA_MAX = 2000.f;    // 扭矩最大编码值
    static constexpr float TRQE_MAX = 4.2f;           // 扭矩最大值
    static constexpr float CURR_DATA_MAX = 2048.f;    // 电流最大编码值
    static constexpr float CURR_MAX = 14.0625f;       // 电流最大值
    static constexpr float TORQ_CONSTANT = 0.32f;     // 转矩常数
    static constexpr float REDUCTION_RATIO = 1.f;     // 减速比
    static constexpr float ENCODER_SPAN = 65535.f;    // 编码器范围
    
    // CAN ID定义
    static constexpr uint16_t TX_TORQUE_ID = 0x280U;  // 扭矩控制发送ID
    static constexpr uint16_t TX_SPEED_ID = 0x281U;   // 速度控制发送ID
    static constexpr uint16_t TX_ANGLE_ID = 0x282U;   // 角度控制发送ID
    static constexpr uint16_t TX_MIX_ID = 0x288U;     // 混合控制发送ID
    static constexpr uint16_t RX_BASE_ID = 0x140U;    // 接收基础ID/
    
    // 构造函数和析构函数
    MF9025(const char _name[16], InitConfig_s _config, uint8_t index);
    ~MF9025() override = default;
};

} // namespace PINYMOTOR::LKMOTOR