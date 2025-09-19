#pragma once

#include "../../Base/MotorBase.hpp"

namespace PINYMOTOR::DJIMOTOR {
#pragma pack(push, 1)
struct Msg_s {
    int16_t cmd[4];
};
struct Feedback_s {
    uint16_t rawAng;
    int16_t rawRpm;
    int16_t current;
    uint8_t temperature;
};
#pragma pack(pop)
struct Status_s {
    float voltTxCodeSpan;
    float currTxCodeSpan;
    float currRxCodeSpan;
    float voltMax; // V
    float currMax; // A
    float torqMax; // Nm
    float Kn;      // Nm/A

    Status_s &operator=(const Status_s &_other);
};

class DJIMotor : public QuadMotorBase {
    using Base = QuadMotorBase;

    using ConvertFunc = void (DJIMotor::*)();

private:
    RxBus_s::CANRxBuf_s<8> rxBuf_ = {}; // buffer for received data

    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len);
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf);
    MotorTypeDef_e ctrl();

    ConvertFunc convert = &DJIMotor::convertDefault;

    void convertQuadCurr();
    void convertQuadVolt();
    void convertDefault();

    void serializeMsg(int16_t _ctrlCmd);

    void overrideReductionRatio(float _newReductionRatio) final;

protected:
    /**
     * @brief Register the receive callback function
     * 
     */
    void registerRecvCallback();
    /**
     * @brief Cancel the receive callback function
     * 
     */
    void cancelRecvCallback();
    /**
     * @brief Update the control ID based on the current work mode
     * 
     */
    void updateCtrlMode();

    Status_s status_;
    uint16_t ctrlId_ = 0xFFFF; // sendId - depends on work mode

public:
    DJIMotor(const char _name[16], InitConfig_s _config);
    ~DJIMotor() override;
    /**
     * @brief Override the status of the motor
     * 
     * @param _newStats 
     */
    void overrideStats(const Status_s &_newStats);

    /**
     * @brief Get the CAN ID of the motor
     * 
     * @return uint16_t 
     */
    uint16_t canId() const; // QuadMotor's canId is fixed

    /**
     * @brief Get the master ID of the motor
     * 
     * @return uint16_t 
     */
    uint16_t masterId() const;

    /**
     * @brief Update the motor state
     * 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e update() final;
};

} // namespace PINYMOTOR::DJIMOTOR
