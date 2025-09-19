#pragma once

#include "IMotor.hpp"

#include <cstdint>
#include <cstring>

#include <unordered_map>

namespace PINYMOTOR::DMMOTOR {
enum class RegId_e : uint8_t {
    DM_REG_UV_VALUE = 0u,   // 低压保护值	RW	(10.0,3.4E38]	float
    DM_REG_KT_VALUE = 1u,   // 扭矩系数	RW	[0.0,3.4E38]	float
    DM_REG_OT_VALUE = 2u,   // 过温保护值	RW	[80.0,200)	float
    DM_REG_OC_VALUE = 3u,   // 过流保护值	RW	(0.0,1.0)	float
    DM_REG_ACC = 4u,        // 加速度	RW	(0.0,3.4E38)	float
    DM_REG_DEC = 5u,        // 减速度	RW	[-3.4E38,0.0)	float
    DM_REG_MAX_SPD = 6u,    // 最大速度	RW	(0.0,3.4E38]	float
    DM_REG_MST_ID = 7u,     // 反馈ID	RW	[0,0x7FF]	uint32
    DM_REG_ESC_ID = 8u,     // 接收ID	RW	[0,0x7FF]	uint32
    DM_REG_TIMEOUT = 9u,    // 超时警报时间	RW	[0,2^32-1]	uint32
    DM_REG_CTRL_MODE = 10u, // 控制模式	RW	[1,4]	uint32
    DM_REG_DAMP = 11u,      // 电机粘滞系数	RO	/	float
    DM_REG_TINERTIA = 12u,  // 电机转动惯量	RO	/	float
    DM_REG_HW_VER = 13u,    // 保留	RO	/	uint32
    DM_REG_SW_VER = 14u,    // 软件版本号	RO	/	uint32
    DM_REG_SN = 15u,        // 保留	RO	/	uint32
    DM_REG_NPP = 16u,       // 电机极对数	RO	/	uint32
    DM_REG_RS = 17u,        // 电机相电阻	RO	/	float
    DM_REG_LS = 18u,        // 电机相电感	RO	/	float
    DM_REG_FLUX = 19u,      // 电机磁链值	RO	/	float
    DM_REG_GR = 20u,        // 齿轮减速比	RO	/	float
    DM_REG_PMAX = 21u,      // 位置映射范围	RW	(0.0,3.4E38]	float
    DM_REG_VMAX = 22u,      // 速度映射范围	RW	(0.0,3.4E38]	float
    DM_REG_TMAX = 23u,      // 扭矩映射范围	RW	(0.0,3.4E38]	float
    DM_REG_I_BW = 24u,      // 电流环控制带宽	RW	[100.0,10000.0]	float
    DM_REG_KP_ASR = 25u,    // 速度环Kp	RW	[0.0,3.4E38]	float
    DM_REG_KI_ASR = 26u,    // 速度环Ki	RW	[0.0,3.4E38]	float
    DM_REG_KP_APR = 27u,    // 位置环Kp	RW	[0.0,3.4E38]	float
    DM_REG_KI_APR = 28u,    // 位置环Ki	RW	[0.0,3.4E38]	float
    DM_REG_OV_VALUE = 29u,  // 过压保护值	RW	TBD	float
    DM_REG_GREF = 30u,      // 齿轮力矩效率	RW	(0.0,1.0]	float
    DM_REG_DETA = 31u,      // 速度环阻尼系数	RW	[1.0,30.0]	float
    DM_REG_V_BW = 32u,      // 速度环滤波带宽	RW	(0.0,500.0)	float
    DM_REG_IQ_C1 = 33u,     // 电流环增强系数	RW	[100.0,10000.0]	float
    DM_REG_VL_C1 = 34u,     // 速度环增强系数	RW	(0.0,10000.0]	float
    DM_REG_CAN_BR = 35u,    // CAN波特率代码	RW	[0,4]	uint32
    DM_REG_SUB_VER = 36u,   // 子版本号	RO	/	uint32
    DM_REG_U_OFF = 50u,     // u相偏置	RO	　	float
    DM_REG_V_OFF = 51u,     // v相偏置	RO	　	float
    DM_REG_K1 = 52u,        // 补偿因子1	RO	　	float
    DM_REG_K2 = 53u,        // 补偿因子2	RO	　	float
    DM_REG_M_OFF = 54u,     // 角度偏移	RO	　	float
    DM_REG_DIR = 55u,       // 方向	RO	　	float
    DM_REG_P_M = 80u,       // 电机位置	RO	　	float
    DM_REG_XOUT = 81u,      // 输出轴位置	RO	　	float
};

enum class ErrorCode_e : uint8_t {
    MOTOR_DISABLE = 0x0u,
    MOTOR_ENABLE = 0x1u,
    OVER_VOLTAGE = 0x8u,
    LOW_VOLTAGE = 0x9u,
    OVER_CURRENT = 0xAu,
    MOS_OVER_HEAT = 0xBu,
    ROTOR_OVER_HEAT = 0xCu,
    COMMUNICATION_LOSS = 0xDu,
    OVERLOAD = 0xEu,
};

#pragma pack(push, 1)
union RegValue_u {
    float value;
    uint8_t dat[4];
};

struct Reg_s {
    RegId_e regId;
    uint8_t dat[4];
    bool isWrite;
    bool isRead;
    bool isStorage;
};

struct Feedback_s {
    uint8_t ID : 4;
    ErrorCode_e errorCode : 4;
    uint16_t rawAng : 14;
    uint16_t rawVel : 12;
    uint16_t torque : 12;
    uint8_t mosTemperature : 8;
    uint8_t rotorTemperature : 8;
};

struct MITMsg_s {
    uint16_t exptScale : 16;
    uint16_t exptVel : 12;
    uint16_t Kp : 12;
    uint16_t Kd : 12;
    uint16_t torqueForward : 12;
};

struct EMITMsg_s {
    float exptScale;
    uint16_t exptVelX100 : 16;
    uint16_t imaxX10000 : 16;
};
struct PDESVDESMsg_s {
    float exptScale;
    float exptVel;
};

struct VDESMsg_s {
    float exptVel;
    float reserved;
};
#pragma pack(pop)

struct Status_s {
    float PMax;
    float VMax;
    float TMax;
    float MITKpMax;
    float MITKdMax;
    float currTxCodeSpan;
    float currMax; // A
    float torqMax; // Nm
    float Kn;      // Nm/A

    Status_s &operator=(const Status_s &_other);
};

class DMMotor : public IMotor {
    using Base = IMotor;
    using RegMap = std::unordered_map<RegId_e, Reg_s *>;

    using TxBus = TxBus_s::CANTxBuf_s<8>;

    using ConvertFunc = TxBus (DMMotor::*)();

private:
    RxBus_s::CANRxBuf_s<8> rxBuf_ = {}; // buffer for received data

    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len);
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf);
    MotorTypeDef_e ctrl();

    ConvertFunc convert = &DMMotor::convertDefault;

    TxBus convertMitTt();
    TxBus convertMitVdes();
    TxBus convertMitVdesPdes();
    TxBus convertPdesVdes();
    TxBus convertVdes();
    TxBus convertEmit();
    TxBus convertDefault();

    TxBus serializeMITMsg(MITMsg_s &_msgMIT);

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

    std::unordered_map<RegId_e, Reg_s *> regObjList_;
    std::unordered_map<RegId_e, RegValue_u *> regValueList_;
    std::unordered_map<RegId_e, uint8_t[4]> preRegValue_;

    float MITKp_ = 0;
    float MITKd_ = 0;

    ErrorCode_e errorCode_;

    uint16_t ctrlId_ = 0XFFFF; // sendId - depends on work mode

public:
    DMMotor(const char _name[16], InitConfig_s _config);
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
};
} // namespace PINYMOTOR::DMMOTOR
