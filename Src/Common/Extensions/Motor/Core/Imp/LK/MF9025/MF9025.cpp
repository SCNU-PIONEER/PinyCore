#include "MF9025.hpp"
#include "StmLog.hpp"
#include <cstring>

using namespace PINYMOTOR;
using namespace LKMOTOR;

// 构造函数
MF9025::MF9025(const char _name[16], InitConfig_s _config, uint8_t index)
    : LKMotor(_name, std::move(_config), index) {
    strcpy(this->model_.name, "LK-MF9025");
    
    LOG::info("MF9025", 
              "%s: MF9025 instance created, rxBaseId:0x%hx, index:%d",
              this->name_, RX_BASE_ID, index);
}

// 检查基础配置
MotorTypeDef_e MF9025::checkBaseConfig() {
    MotorTypeDef_e rslt = 0;

    // 检查通信类型，仅支持CAN
    if (this->comType_ != PINYMOTOR::ComType_e::CAN) {
        rslt |= 1;
        LOG::error("MF9025", "%s: Only CAN communication is supported", this->name_);
    }

    // 检查工作模式是否支持
    if (this->workMode_ != WorkMode_e::QUAD_CURR &&
        this->workMode_ != WorkMode_e::QUAD_VEL &&
        this->workMode_ != WorkMode_e::QUAD_POS &&
        this->workMode_ != WorkMode_e::QUAD_MIX) {
        rslt |= 1;
        LOG::error("MF9025", "%s: Unsupported work mode", this->name_);
    }

    // 检查电机索引是否在有效范围内(0-3)
    if (motorIndex_ > 3) {
        rslt |= 1;
        LOG::error("MF9025", "%s: Motor index must be 0-3, got %d", this->name_, motorIndex_);
    }

    // 检查发送频率是否在允许范围内
    if (this->txFreq_ > 1000) {
        rslt |= 1;
        LOG::error("MF9025", "%s: Maximum transmit frequency is 1000Hz", this->name_);
    }

    return rslt;
}

// 初始化电机模型参数
void MF9025::initModelParams() {
    // 设置电机模型参数
    this->model_.measureMax = static_cast<int>(ENCODER_SPAN);
    this->model_.measureMin = 0;
    this->model_.reductionRatio = REDUCTION_RATIO;
    this->model_.rxBaseId = RX_BASE_ID;
    
    // 根据工作模式设置发送基础ID
    switch (this->workMode_) {
        case WorkMode_e::QUAD_CURR:
            this->model_.txBaseId = TX_TORQUE_ID;
            break;
        case WorkMode_e::QUAD_VEL:
            this->model_.txBaseId = TX_SPEED_ID;
            break;
        case WorkMode_e::QUAD_POS:
            this->model_.txBaseId = TX_ANGLE_ID;
            break;
        case WorkMode_e::QUAD_MIX:
            this->model_.txBaseId = TX_MIX_ID;
            break;
        default:
            this->model_.txBaseId = TX_TORQUE_ID;
            break;
    }
    
    // 初始化状态参数
    status_ = Status_s(
        TRQE_DATA_MAX,    // 扭矩最大编码值
        TRQE_MAX,         // 扭矩最大值
        CURR_DATA_MAX,    // 电流最大编码值
        CURR_MAX,         // 电流最大值
        TORQ_CONSTANT,    // 转矩常数
        REDUCTION_RATIO,  // 减速比
        ENCODER_SPAN,     // 编码器范围
        TX_TORQUE_ID,     // 扭矩控制发送ID
        TX_SPEED_ID,      // 速度控制发送ID
        TX_ANGLE_ID,      // 角度控制发送ID
        TX_MIX_ID,        // 混合控制发送ID
        RX_BASE_ID        // 接收基础ID
    );
}