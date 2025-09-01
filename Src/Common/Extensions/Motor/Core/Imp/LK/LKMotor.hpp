#pragma once

#include "../../Base/MotorBase.hpp"
#include "Bsp_can.hpp"
#include "StmLog.hpp"

namespace PINYMOTOR::LKMOTOR {

#pragma pack(push, 1)
/**
 * @brief 电机反馈数据结构
 */
struct Feedback_s {
    uint8_t cmd;         // 命令字节
    uint8_t temperature; // 电机温度
    int16_t current;     // 转矩电流编码值
    int16_t speed;       // 转子角速度(°/s)
    uint16_t angle;      // 编码器位置
};

/**
 * @brief 电机发送数据结构
 */
struct Transmit_s {
    int16_t data[4]; // 4个电机的16位数据
    bool sendFlag;   // 发送标志
};

/**
 * @brief 电机状态参数结构
 */
struct Status_s {
    // 转换参数
    float torqueDataMax;  // 扭矩最大编码值
    float torqueMax;      // 扭矩最大值
    float currentDataMax; // 电流最大编码值
    float currentMax;     // 电流最大值
    float torqueConstant; // 转矩常数
    float reductionRatio; // 减速比
    float encoderSpan;    // 编码器范围

    // 硬件参数
    uint16_t txTorqueId; // 扭矩控制发送ID
    uint16_t txSpeedId;  // 速度控制发送ID
    uint16_t txAngleId;  // 角度控制发送ID
    uint16_t txMixId;    // 混合控制发送ID
    uint16_t rxBaseId;   // 接收基础ID

    Status_s() = default;
    Status_s(float tqDataMax, float tqMax, float currDataMax, float currMax,
             float tqConst, float rr, float encSpan, uint16_t txTqId,
             uint16_t txSpdId, uint16_t txAngId, uint16_t txMixId,
             uint16_t rxBase)
            : torqueDataMax(tqDataMax)
            , torqueMax(tqMax)
            , currentDataMax(currDataMax)
            , currentMax(currMax)
            , torqueConstant(tqConst)
            , reductionRatio(rr)
            , encoderSpan(encSpan)
            , txTorqueId(txTqId)
            , txSpeedId(txSpdId)
            , txAngleId(txAngId)
            , txMixId(txMixId)
            , rxBaseId(rxBase)
    {
    }
};
#pragma pack(pop)

class LKMotor : public QuadMotorBase {
    using Base = QuadMotorBase;

protected:
    RxBus_s::CANRxBuf_s<8> rxBuf_ = {}; // 接收缓冲区
    Transmit_s txBuf_ = {};             // 发送缓冲区
    Status_s status_;                   // 电机状态参数
    uint16_t currentTxId_;              // 当前发送ID
    uint8_t motorIndex_;                // 电机索引(0-3)

    // 保护成员函数
    MotorTypeDef_e send(uint16_t sendId, uint8_t *txBuf, uint8_t len);
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s<8> &rxBuf);
    MotorTypeDef_e ctrl();
    void updateTxId();

    /**
     * @brief 检查基础配置
     */
    virtual MotorTypeDef_e checkBaseConfig() = 0;

    /**
     * @brief 初始化电机模型参数
     */
    virtual void initModelParams() = 0;

public:
    LKMotor(const char name[16], InitConfig_s config, uint8_t index);
    ~LKMotor() override;

    /**
     * @brief 覆盖电机状态参数
     */
    void overrideStats(const Status_s &newStats);

    /**
     * @brief 获取电机CAN ID
     */
    uint16_t canId() const override;

    /**
     * @brief 获取电机主ID
     */
    uint16_t masterId() const override;

    /**
     * @brief 获取电机唯一标识
     */
    uint16_t uid() final;

    /**
     * @brief 更新电机状态
     */
    MotorTypeDef_e update() final;

    /**
     * @brief 挂载电机数据到广播帧
     */
    void mountToBroadcastFrame(Transmit_s &broadcastFrame);

    // 注册和取消回调函数
    void registerRecvCallback() override;
    void cancelRecvCallback() override;
};

} // namespace PINYMOTOR::LKMOTOR