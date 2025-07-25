#pragma once

#include "IMotor.hpp"
#include "UTMotorMsg.hpp"
#include "Bsp_dma.hpp"

namespace PINYMOTOR::UTMOTOR {

struct Status_s {
    float PMax;
    float VMax;
    float TMax;
    float KpMax;
    float KdMax;
    float currMax;  // A
    float torqMax;  // Nm
    float speedMax; //rad/s
    float Kn;       // Nm/A
    ErrorStatus_e error_;

    Status_s &operator=(const Status_s &_other);
};

class UTMotor : public IMotor {
private:
    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len);
    MotorTypeDef_e parse(uint8_t *_rxBuf);
    MotorTypeDef_e ctrl();

protected:
    uint16_t ctrlId_ = 0xFFFF;
    float kp_; //电机内置pid
    float kd_;
    uint8_t *txBuf_ = nullptr;
    uint8_t *rxBuf_ = nullptr;
    Status_s status_;

    DMA_HandleTypeDef *dmaHandle_;

    void registerRecvCallback();
    void convert(TransmitMsg_s &_txBuf, const Cmd_s &_cmd);

public:
    UTMotor(const char _name[16], InitConfig_s _config,
            DMA_HandleTypeDef *_dmaHandle);
    ~UTMotor() override;

    void overrideStats(const Status_s &_newStats);

    bool isEnable() const;
    uint16_t getSendId() const;
    uint16_t getReceiveId() const;

    uint16_t uid() final;

    MotorTypeDef_e update() final;

    void setKp(const float _kp);
    void setKd(const float _kd);

    MotorTypeDef_e enable();
    MotorTypeDef_e disable();
};

} // namespace PINYMOTOR::UTMOTOR
