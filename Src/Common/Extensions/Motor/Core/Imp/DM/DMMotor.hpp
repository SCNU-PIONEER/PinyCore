#pragma once

#include "../../Base/MotorBase.hpp"

#include <cstdint>
#include <cstring>

#include <unordered_map>

namespace PINYMOTOR {

enum class DMMotorRegId_e {
    DM_REG_UV_Value  = 0u, // 低压保护值	RW	(10.0,3.4E38]	float
    DM_REG_KT_Value  = 1u, // 扭矩系数	RW	[0.0,3.4E38]	float
    DM_REG_OT_Value  = 2u, // 过温保护值	RW	[80.0,200)	float
    DM_REG_OC_Value  = 3u, // 过流保护值	RW	(0.0,1.0)	float
    DM_REG_ACC       = 4u, // 加速度	RW	(0.0,3.4E38)	float
    DM_REG_DEC       = 5u, // 减速度	RW	[-3.4E38,0.0)	float
    DM_REG_MAX_SPD   = 6u, // 最大速度	RW	(0.0,3.4E38]	float
    DM_REG_MST_ID    = 7u, // 反馈ID	RW	[0,0x7FF]	uint32
    DM_REG_ESC_ID    = 8u, // 接收ID	RW	[0,0x7FF]	uint32
    DM_REG_TIMEOUT   = 9u, // 超时警报时间	RW	[0,2^32-1]	uint32
    DM_REG_CTRL_MODE = 10u, // 控制模式	RW	[1,4]	uint32
    DM_REG_Damp      = 11u, // 电机粘滞系数	RO	/	float
    DM_REG_Inertia   = 12u, // 电机转动惯量	RO	/	float
    DM_REG_hw_ver    = 13u, // 保留	RO	/	uint32
    DM_REG_sw_ver    = 14u, // 软件版本号	RO	/	uint32
    DM_REG_SN        = 15u, // 保留	RO	/	uint32
    DM_REG_NPP       = 16u, // 电机极对数	RO	/	uint32
    DM_REG_Rs        = 17u, // 电机相电阻	RO	/	float
    DM_REG_Ls        = 18u, // 电机相电感	RO	/	float
    DM_REG_Flux      = 19u, // 电机磁链值	RO	/	float
    DM_REG_Gr        = 20u, // 齿轮减速比	RO	/	float
    DM_REG_PMAX      = 21u, // 位置映射范围	RW	(0.0,3.4E38]	float
    DM_REG_VMAX      = 22u, // 速度映射范围	RW	(0.0,3.4E38]	float
    DM_REG_TMAX      = 23u, // 扭矩映射范围	RW	(0.0,3.4E38]	float
    DM_REG_I_BW      = 24u, // 电流环控制带宽	RW	[100.0,10000.0]	float
    DM_REG_KP_ASR    = 25u, // 速度环Kp	RW	[0.0,3.4E38]	float
    DM_REG_KI_ASR    = 26u, // 速度环Ki	RW	[0.0,3.4E38]	float
    DM_REG_KP_APR    = 27u, // 位置环Kp	RW	[0.0,3.4E38]	float
    DM_REG_KI_APR    = 28u, // 位置环Ki	RW	[0.0,3.4E38]	float
    DM_REG_OV_Value  = 29u, // 过压保护值	RW	TBD	float
    DM_REG_GREF      = 30u, // 齿轮力矩效率	RW	(0.0,1.0]	float
    DM_REG_Deta      = 31u, // 速度环阻尼系数	RW	[1.0,30.0]	float
    DM_REG_V_BW      = 32u, // 速度环滤波带宽	RW	(0.0,500.0)	float
    DM_REG_IQ_c1     = 33u, // 电流环增强系数	RW	[100.0,10000.0]	float
    DM_REG_VL_c1     = 34u, // 速度环增强系数	RW	(0.0,10000.0]	float
    DM_REG_can_br    = 35u, // CAN波特率代码	RW	[0,4]	uint32
    DM_REG_sub_ver   = 36u, // 子版本号	RO	/	uint32
    DM_REG_u_off     = 50u, // u相偏置	RO	　	float
    DM_REG_v_off     = 51u, // v相偏置	RO	　	float
    DM_REG_k1        = 52u, // 补偿因子1	RO	　	float
    DM_REG_k2        = 53u, // 补偿因子2	RO	　	float
    DM_REG_m_off     = 54u, // 角度偏移	RO	　	float
    DM_REG_dir       = 55u, // 方向	RO	　	float
    DM_REG_p_m       = 80u, // 电机位置	RO	　	float
    DM_REG_xout      = 81u, // 输出轴位置	RO	　	float   
};
// 寄存器列表


#pragma pack(push, 1)
struct DMMotorReg_s {
    DMMotorRegId_e regId;
    uint8_t dat[4];
    bool isWrite;
    bool isRead;
    bool isStorage;
};
enum class DMMotorErrorCode_e{
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

struct DMMotorFeedback_s {
    uint8_t ID : 4;
    DMMotorErrorCode_e errorCode : 4;
    uint16_t rawScale : 14;
    uint16_t rawVel : 12;
    uint16_t torque : 12;
    uint8_t mosTemperature : 8;
    uint8_t rotorTemperature : 8;
};

struct DMMITMsg_s {
    int16_t exptScale : 16;
    int16_t exptVel : 12;
    int16_t Kp : 12;
    int16_t Kd : 12;
    int16_t torqueOffset : 12;
};

struct DMEMITMsg_s {
    float exptScale;
    uint16_t exptVelX100 : 16;
    uint16_t imaxX10000 : 16;
};
struct DMPDESVDESMsg_s {
    float exptScale;
    float exptVel;
};

struct DMVDESMsg_s {
    float exptVel;
    float reserved;
} ;
#pragma pack(pop)

struct DMMotorStats_s {
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

    DMMotorStats_s& operator=(const DMMotorStats_s& _other) {
        if (this != &_other) {
            PMax = _other.PMax;
            VMax = _other.VMax;
            TMax = _other.TMax;
            MITKpMax = _other.MITKpMax;
            MITKdMax = _other.MITKdMax;
            currTxCodeSpan = _other.currTxCodeSpan;
            currRated = _other.currRated;
            torqRated = _other.torqRated;
            currMax = _other.currMax;
            torqMax = _other.torqMax;
            torqConstant = _other.torqConstant;
        }
        return *this;
    }
};

template <typename Derived> class DMMotor : public MotorBase<Derived> {
    // 定义一个模板类DMMotor，继承自MotorBase类，并使用Derived作为模板参数
    using Base = MotorBase<Derived>;

private:
    // 注册解析函数
    void registerRecvCallback();

protected:
    // 定义一个保护成员变量stats，类型为DMMotorStats_s
    DMMotorStats_s stats_;

    // 正在操作的寄存器对象
    std::unordered_map<DMMotorRegId_e,DMMotorReg_s*> regObjList_;

    // 定义两个保护成员变量MITKp和MITKd，类型为float
    float MITKp_ = 0;
    float MITKd_ = 0;

    // 达妙电机反馈自身状态
    DMMotorErrorCode_e errorCode_;

    struct Cmd_s {
        bool SW;
        bool prevSW;
        struct {
            float torq;
            float speed;
            float pos;
        };
        void clear() {
            SW = false;
            torq = 0;
            speed = 0;
            pos = 0;
        }
        void updateSW(bool _sw)
        {
            if (_sw != prevSW) {
                SW = _sw;
                prevSW = _sw;
            }
        }
    } cmd_;

    uint16_t ctrlId_ = 0x00; // 控制ID - 根据工作模式变化
    
public:
    // 构造函数，接受一个InitConfig_s类型的参数_config，并调用基类的构造函数
    inline DMMotor(const char _name[16], InitConfig_s _config)
            : Base(_name, _config)
    {
        cmd_.clear();
        registerRecvCallback(); // 注册解析函数
    }
    
    // 重写电机属性
    inline void overrideStats(const DMMotorStats_s& _newStats)
    {
        // 将新的电机属性赋值给stats
        stats_ = _newStats;
    }

    MotorTypeDef_e _cmd_(MotorCmdType_e _cmd, float _cmdData);
    MotorTypeDef_e _cmd_(MotorCmdType_e _cmd);

    inline bool isEnable() const { return cmd_.SW; }

    inline uint16_t canId() const
    {
        return this->model_.txBaseId + this->offsetId_;
    }
    
    inline uint16_t masterId() const
    {
        return this->model_.rxBaseId + this->offsetId_;
    }

    inline uint16_t _uid_() { return masterId(); }

    MotorTypeDef_e _send_(uint8_t *_txBuf, uint8_t _len);

    MotorTypeDef_e _parse_(uint8_t *_rxBuf);
    
    MotorTypeDef_e _ctrl_();

    inline void setMITKp(float _kp) { MITKp_ = _kp; }
    inline void setMITKd(float _kd) { MITKd_ = _kd; }

    MotorTypeDef_e enable();

    MotorTypeDef_e disable();

    MotorTypeDef_e registerReg(DMMotorReg_s *_regObj);

    MotorTypeDef_e cancelReg(DMMotorRegId_e regId);

    MotorTypeDef_e writeReg(DMMotorRegId_e _regId, uint8_t dat[4]);

    MotorTypeDef_e readReg(DMMotorRegId_e _regId);
    
    MotorTypeDef_e storageReg(DMMotorRegId_e _regId);
};
}
