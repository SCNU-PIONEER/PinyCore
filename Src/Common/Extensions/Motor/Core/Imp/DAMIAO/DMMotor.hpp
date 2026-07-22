#pragma once

#include "IMotor.hpp"

#include "DMMotorMsg.hpp"

#include <unordered_map>
namespace PINYMOTOR::DMMOTOR {

enum class WorkMode_e : MotorTypeDef_e { UNKNOWN, MIT_TT, MIT_VDES, MIT_VDESPDES, PDESVDES, VDES, EMIT };

class DMMotor : public IMotor {
    using Base = IMotor;
    using RegMap = std::unordered_map<RegId_e, Reg_s *>;

    using TxBus = TxBus_s::CANTxBuf_s<8>;

    using ConvertFunc = void (DMMotor::*)(TxBus &);

private:
    RxBus_s::CANRxBuf_s<8> rxBuf_ = {}; // buffer for received data

    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len);
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf);
    MotorTypeDef_e ctrl();

    ConvertFunc convert = &DMMotor::convertDefault;

    void convertMitTt(TxBus &);
    void convertMitVdes(TxBus &);
    void convertMitVdesPdes(TxBus &);
    void convertPdesVdes(TxBus &);
    void convertVdes(TxBus &);
    void convertEmit(TxBus &);
    void convertDefault(TxBus &);

    void serializeMITMsg(MITMsg_s &_msgMIT, TxBus &_txBuf);

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

    std::unordered_map<RegId_e, Reg_s *> regObjList_;
    std::unordered_map<RegId_e, RegValue_u *> regValueList_;
    std::unordered_map<RegId_e, uint8_t[4]> preRegValue_;

    float MITKp_ = 0;
    float MITKd_ = 0;

    ErrorCode_e errorCode_;

    uint16_t ctrlId_ = 0XFFFF; // sendId - depends on work mode

public:
    DMMotor(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);
    ~DMMotor() override;

    /**
     * @brief Override the status of the motor
     * 
     * @param _newStats 
     */
    void overrideStats(const Status_s &_newStats);

    /**
     * @brief Check if the motor is enabled
     * 
     * @return true 
     * @return false 
     */
    bool isEnable() const;

    /**
     * @brief Get the CAN ID of the motor
     * 
     * @return uint16_t 
     */
    uint16_t canId() const;

    /**
     * @brief Update the motor state
     * 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e update() final;

    /**
     * @brief Set the MIT Kp value
     * 
     * @param _kp 
     */
    void setMITKp(float _kp);

    /**
     * @brief Set the MIT Kd value
     * 
     * @param _kd 
     */
    void setMITKd(float _kd);

    /**
     * @brief Enable the motor
     * 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e enable();

    /**
     * @brief Disable the motor
     * 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e disable();

    /**
     * @brief Clear the error code of the motor
     * 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e clearError();

    /**
     * @brief Register a register object
     * 
     * @param _regObj 
     * @param _regValue 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e registerReg(Reg_s *_regObj, RegValue_u *_regValue);

    /**
     * @brief Cancel a register object
     * 
     * @param _regId 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e cancelReg(RegId_e _regId);

    /**
     * @brief Write a single register
     * 
     * @param _regId 
     * @param _dat 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e writeOneReg(RegId_e _regId, uint8_t _dat[4]);

    /**
     * @brief Read a single register
     * 
     * @param _regId 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e readOneReg(RegId_e _regId);

    /**
     * @brief Store a single register
     * 
     * @param _regId 
     * @return MotorTypeDef_e 
     */
    MotorTypeDef_e storageOneReg(RegId_e _regId);

    /**
     * @brief Update and mark the registers that need to be written
     * 
     * @return MotorTypeDef_e 
     */

    MotorTypeDef_e updateRegDat();

    /**
     * @brief Change the control mode dynamically
     * 
     * @param WorkMode_e
     */
    void switchCtrlMode(WorkMode_e _workMode);

    /**
     * @brief Get dmmotor errorcode
     * 
     * @return ErrorCode_e
     */
    ErrorCode_e getErrorcode() { return errorCode_; }
};

} // namespace PINYMOTOR::DMMOTOR
