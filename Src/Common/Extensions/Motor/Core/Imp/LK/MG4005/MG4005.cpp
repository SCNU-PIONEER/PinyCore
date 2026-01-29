#include "MG4005.hpp"

#include "LKMotor.hpp"

#include "StmLog.hpp"

#include <cstring>

using namespace PINYMOTOR;
using namespace LKMOTOR;

/**
 * @brief 构造函数
 */
MG4005::MG4005(const char _name[16], InitConfig_s _config) : LKMotor(_name, _config)
{
    this->regInfo_.model.reductionRatio = REDUCTION_RATIO;
    this->regInfo_.model.measureMax = ENCODER_SPAN;
    this->regInfo_.model.measureMin = 0;
    this->checkBaseConfig();
    this->initModelParams();

    LOG::info("MG4005", "%s: Instance created (index: %d, RX Base ID: 0x%hx)", this->regInfo_.name,
              regInfo_.model.txBaseId, regInfo_.model.rxBaseId);
}

MotorTypeDef_e MG4005::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;
    if (this->regInfo_.comType != ComType_e::CAN) {
        rslt = 1;
        LOG::error("MG4005", "%s: Only CAN communication supported", this->regInfo_.name);
    }
    if (this->regInfo_.workMode != WorkMode_e::QUAD_CURR) {
        rslt = 1;
        LOG::error("MG4005", "%s: Only QUAD_CURR mode supported", this->regInfo_.name);
    }
    if (this->motorIndex_ < 1 || this->motorIndex_ > 4) {
        rslt = 1;
        LOG::error("MG4005", "%s: Index must be 1-4 (got %d)", this->regInfo_.name, this->motorIndex_);
    }
    if (this->AUX_.txFreq > 1000) {
        rslt = 1;
        LOG::error("MG4005", "%s: TX freq max 1000Hz (got %.0fHz)", this->regInfo_.name, this->AUX_.txFreq);
    }
    if (this->regInfo_.pComHandle == nullptr) {
        rslt = 1;
        LOG::error("MG4005", "%s: CAN handle is null", this->regInfo_.name);
    }
    return rslt;
}

/**
 * @brief 初始化电机模型参数
 */
void MG4005::initModelParams()
{
    this->status_ = Status_s(TX_CURR_DATA_MAX, // 发送电流最大编码值
                             RX_CURR_DATA_MAX, // 接收电流最大编码值
                             TRQE_MAX,         // 扭矩最大值
                             TX_CURR_MAX,      // 接收电流最大编码值
                             RX_CURR_MAX,      // 电流最大值
                             TORQ_CONSTANT     // 转矩常数
    );

    this->data_.zeroAng = 0.0f;
    this->data_.angLast = 0.0f;
    this->data_.multipCirAng = 0.0f;
    this->globalState = GlobalState_e::OFFLINE;

    LOG::info("MG4005", "%s: Model params init: RR=%.1f, EncoderSpan=%.0f", this->regInfo_.name, REDUCTION_RATIO,
              ENCODER_SPAN);
}
