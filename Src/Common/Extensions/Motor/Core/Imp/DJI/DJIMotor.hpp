#pragma once

#include "../../Base/MotorBase.hpp"

#include "DJIMotorMsg.hpp"

namespace PINYMOTOR::DJIMOTOR {

enum class WorkMode_e : MotorTypeDef_e { UNKNOWN, QUAD_CURR, QUAD_VOLT };
class DJIMotor : public QuadMotorBase {
    using Base = QuadMotorBase;

    using ConvertFunc = void (DJIMotor::*)();

private:
    RxBus_s::CANRxBuf_s<8> rxBuf_{}; // buffer for received data

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
    void registerRecvCallback(uint16_t _rxId);
    /**
     * @brief Cancel the receive callback function
     * 
     */
    void cancelRecvCallback(uint16_t _rxId);
    /**
     * @brief Update the control ID based on the current work mode
     * 
     */
    void updateCtrlMode();

    Status_s status_;
    WorkMode_e workMode_ = WorkMode_e::UNKNOWN;
    bool isSupportMode(WorkMode_e _mode) const;
    uint16_t ctrlId_ = 0xFFFF; // sendId - depends on work mode

public:
    DJIMotor(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);
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
     * @brief Update the motor state
     * 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e update() final;
};

} // namespace PINYMOTOR::DJIMOTOR
