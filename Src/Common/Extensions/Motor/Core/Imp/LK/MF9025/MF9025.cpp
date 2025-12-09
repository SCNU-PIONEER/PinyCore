#include "MF9025.hpp"

#include "LKMotor.hpp"

#include "StmLog.hpp"

#include <cstring>

using namespace PINYMOTOR;
using namespace LKMOTOR;

/**
 * @brief 构造函数
 */
MF9025::MF9025(const char _name[16], InitConfig_s _config) : LKMotor(_name, _config)
{
    this->regInfo_.model.rxBaseId = RX_BASE_ID;
    this->regInfo_.model.txBaseId = TX_CURR_ID;
    this->regInfo_.model.reductionRatio = REDUCTION_RATIO;
    this->regInfo_.model.measureMax = static_cast<int>(ENCODER_SPAN);
    this->regInfo_.model.measureMin = 0;

    MotorTypeDef_e initResult = this->init();
    if (initResult != MOTOR_OK) {
        LOG::error("MF9025", "%s: Init failed", this->regInfo_.name);
    } else {
        LOG::info("MF9025", "%s: Instance created (index: %d, RX Base ID: 0x%hx)", this->regInfo_.name, index,
                  RX_BASE_ID);
    }
}

/**
 * @brief 检查基础配置
 */
MotorTypeDef_e MF9025::checkBaseConfig()
{
    MotorTypeDef_e rslt = MOTOR_OK;

    // 检查通信类型
    if (this->regInfo_.comType != ComType_e::CAN) {
        rslt = MOTOR_ERROR;
        LOG::error("MF9025", "%s: Only CAN communication supported", this->regInfo_.name);
    }

    // 检查工作模式
    if (this->regInfo_.workMode != WorkMode_e::QUAD_CURR) {
        rslt = MOTOR_ERROR;
        LOG::error("MF9025", "%s: Only QUAD_CURR mode supported", this->regInfo_.name);
    }

    // 检查电机索引
    if (this->motorIndex_ > 3) {
        rslt = MOTOR_ERROR;
        LOG::error("MF9025", "%s: Index must be 0-3 (got %d)", this->regInfo_.name, this->motorIndex_);
    }

    // 检查发送频率
    if (this->AUX_.txFreq > 1000) {
        rslt = MOTOR_ERROR;
        LOG::error("MF9025", "%s: TX freq max 1000Hz (got %.0fHz)", this->regInfo_.name, this->AUX_.txFreq);
    }

    // 检查CAN句柄
    if (this->regInfo_.pComHandle == nullptr) {
        rslt = MOTOR_ERROR;
        LOG::error("MF9025", "%s: CAN handle is null", this->regInfo_.name);
    }

    return rslt;
}

/**
 * @brief 初始化电机模型参数
 */
void MF9025::initModelParams()
{
    this->status_ = Status_s(TRQE_DATA_MAX,   // 扭矩最大编码值
                             TRQE_MAX,        // 扭矩最大值
                             CURR_DATA_MAX,   // 电流最大编码值
                             CURR_MAX,        // 电流最大值
                             TORQ_CONSTANT,   // 转矩常数
                             REDUCTION_RATIO, // 减速比
                             ENCODER_SPAN,    // 编码器范围
                             TX_CURR_ID,      // 发送ID
                             RX_BASE_ID       // 接收基础ID
    );

    // 同步基类data_初始值
    this->data_.zeroAng = 0.0f;
    this->data_.angLast = 0.0f;
    this->data_.multipCirAng = 0.0f;
    this->globalState = GlobalState_e::OFFLINE;

    // 日志输出初始化结果
    LOG::info("MF9025", "%s: Model params init: RR=%.1f, EncoderSpan=%.0f", this->regInfo_.name, REDUCTION_RATIO,
              ENCODER_SPAN);
}