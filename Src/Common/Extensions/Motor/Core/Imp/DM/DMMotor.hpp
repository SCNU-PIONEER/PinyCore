#pragma once

#include "../../Base/MotorBase.hpp"

#include <cstdint>
#include <cstring>

#include <unordered_map>

namespace PINYMOTOR {
namespace DMMOTOR {
enum class RegId_e {
    DM_REG_UV_Value = 0u,   // 低压保护值	RW	(10.0,3.4E38]	float
    DM_REG_KT_Value = 1u,   // 扭矩系数	RW	[0.0,3.4E38]	float
    DM_REG_OT_Value = 2u,   // 过温保护值	RW	[80.0,200)	float
    DM_REG_OC_Value = 3u,   // 过流保护值	RW	(0.0,1.0)	float
    DM_REG_ACC = 4u,        // 加速度	RW	(0.0,3.4E38)	float
    DM_REG_DEC = 5u,        // 减速度	RW	[-3.4E38,0.0)	float
    DM_REG_MAX_SPD = 6u,    // 最大速度	RW	(0.0,3.4E38]	float
    DM_REG_MST_ID = 7u,     // 反馈ID	RW	[0,0x7FF]	uint32
    DM_REG_ESC_ID = 8u,     // 接收ID	RW	[0,0x7FF]	uint32
    DM_REG_TIMEOUT = 9u,    // 超时警报时间	RW	[0,2^32-1]	uint32
    DM_REG_CTRL_MODE = 10u, // 控制模式	RW	[1,4]	uint32
    DM_REG_Damp = 11u,      // 电机粘滞系数	RO	/	float
    DM_REG_Inertia = 12u,   // 电机转动惯量	RO	/	float
    DM_REG_hw_ver = 13u,    // 保留	RO	/	uint32
    DM_REG_sw_ver = 14u,    // 软件版本号	RO	/	uint32
    DM_REG_SN = 15u,        // 保留	RO	/	uint32
    DM_REG_NPP = 16u,       // 电机极对数	RO	/	uint32
    DM_REG_Rs = 17u,        // 电机相电阻	RO	/	float
    DM_REG_Ls = 18u,        // 电机相电感	RO	/	float
    DM_REG_Flux = 19u,      // 电机磁链值	RO	/	float
    DM_REG_Gr = 20u,        // 齿轮减速比	RO	/	float
    DM_REG_PMAX = 21u,      // 位置映射范围	RW	(0.0,3.4E38]	float
    DM_REG_VMAX = 22u,      // 速度映射范围	RW	(0.0,3.4E38]	float
    DM_REG_TMAX = 23u,      // 扭矩映射范围	RW	(0.0,3.4E38]	float
    DM_REG_I_BW = 24u,      // 电流环控制带宽	RW	[100.0,10000.0]	float
    DM_REG_KP_ASR = 25u,    // 速度环Kp	RW	[0.0,3.4E38]	float
    DM_REG_KI_ASR = 26u,    // 速度环Ki	RW	[0.0,3.4E38]	float
    DM_REG_KP_APR = 27u,    // 位置环Kp	RW	[0.0,3.4E38]	float
    DM_REG_KI_APR = 28u,    // 位置环Ki	RW	[0.0,3.4E38]	float
    DM_REG_OV_Value = 29u,  // 过压保护值	RW	TBD	float
    DM_REG_GREF = 30u,      // 齿轮力矩效率	RW	(0.0,1.0]	float
    DM_REG_Deta = 31u,      // 速度环阻尼系数	RW	[1.0,30.0]	float
    DM_REG_V_BW = 32u,      // 速度环滤波带宽	RW	(0.0,500.0)	float
    DM_REG_IQ_c1 = 33u,     // 电流环增强系数	RW	[100.0,10000.0]	float
    DM_REG_VL_c1 = 34u,     // 速度环增强系数	RW	(0.0,10000.0]	float
    DM_REG_can_br = 35u,    // CAN波特率代码	RW	[0,4]	uint32
    DM_REG_sub_ver = 36u,   // 子版本号	RO	/	uint32
    DM_REG_u_off = 50u,     // u相偏置	RO	　	float
    DM_REG_v_off = 51u,     // v相偏置	RO	　	float
    DM_REG_k1 = 52u,        // 补偿因子1	RO	　	float
    DM_REG_k2 = 53u,        // 补偿因子2	RO	　	float
    DM_REG_m_off = 54u,     // 角度偏移	RO	　	float
    DM_REG_dir = 55u,       // 方向	RO	　	float
    DM_REG_p_m = 80u,       // 电机位置	RO	　	float
    DM_REG_xout = 81u,      // 输出轴位置	RO	　	float
};

enum class ErrorCode_e {
    MotorDisable = 0x0u,
    MotorEnable = 0x1u,
    OverVoltage = 0x8u,
    LowVoltage = 0x9u,
    OverCurrent = 0xAu,
    MosOverHeat = 0xBu,
    RotorOverHeat = 0xCu,
    CommunicationLoss = 0xDu,
    Overload = 0xEu,
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
    uint16_t rawScale : 14;
    uint16_t rawVel : 12;
    uint16_t torque : 12;
    uint8_t mosTemperature : 8;
    uint8_t rotorTemperature : 8;
};

struct MITMsg_s {
    int16_t exptScale : 16;
    int16_t exptVel : 12;
    int16_t Kp : 12;
    int16_t Kd : 12;
    int16_t torqueOffset : 12;
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
    float currRated;    // A
    float torqRated;    // Nm
    float currMax;      // A
    float torqMax;      // Nm
    float torqConstant; // Nm/A

    Status_s &operator=(const Status_s &_other);
};

class DMMotor : public MotorBase {
    using Base = MotorBase;
    using RegMap = std::unordered_map<RegId_e, Reg_s *>;

private:
    RxBus_s::CANRxBuf_s rxBuf_ = {}; // buffer for received data

protected:
    void registerRecvCallback();
    void cancelRecvCallback();
    void updateCtrlId();

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

    void overrideStats(const Status_s &_newStats);

    bool isEnable() const;
    uint16_t canId() const;
    uint16_t masterId() const;

    uint16_t uid() override final;
    MotorTypeDef_e send(uint16_t _sendId, uint8_t *_txBuf,
                        uint8_t _len) override final;
    MotorTypeDef_e parse(const RxBus_s::CANRxBuf_s &_rxBuf);
    MotorTypeDef_e ctrl() override final;

    void setMITKp(float _kp);
    void setMITKd(float _kd);

    MotorTypeDef_e enable();
    MotorTypeDef_e disable();
    MotorTypeDef_e clearError();

    MotorTypeDef_e registerReg(Reg_s *_regObj, RegValue_u *_regValue);
    MotorTypeDef_e cancelReg(RegId_e regId);
    MotorTypeDef_e writeOneReg(RegId_e _regId, uint8_t dat[4]);
    MotorTypeDef_e readOneReg(RegId_e _regId);
    MotorTypeDef_e storageOneReg(RegId_e _regId);
    MotorTypeDef_e writeReg();
    MotorTypeDef_e readReg();
    MotorTypeDef_e storageReg();
    MotorTypeDef_e updateRegDat();
};
}
}
