#pragma once

#include <cstdint>
#include <memory>

#include "Pid.hpp"


namespace PINYMOTOR {

using MotorTypeDef_e = uint8_t;

enum class MotorCmdType_e : uint8_t {
    SET_MIT,
    SET_POS,
    SET_VEL,
    SET_POSVEL,
    SET_TORQ,
    SET_ELEC,
    ON,
    OFF
};

enum class WorkMode_e : uint8_t {
    TRIP_VOLT,
    QUAD_CURR,
    QUAD_VOLT,
    MIT_TT,
    MIT_VDES,
    MIT_VDESPDES,
    PDESVDES,
    VDES,
    EMIT
};
enum class ComType_e : uint8_t { NONE, FDCAN, CAN, RS485 };

enum class GlobalState_e : uint8_t { UNREGISTER, OFFLINE, ONLINE, ERROR };

enum class MotorErrorCode_e : uint8_t { ERROR_NONE = 0u };

struct CmdBus_s {
    // Basic command structure
    struct {
        MotorCmdType_e cmdType;
        float posCmd;
        float velCmd;
        float torqCmd;
        float elecCmd;
    };
    // auxiliary command structure
    struct {
        // default -1.f means no limit
        float velMax = -1.f;
        // default posMax and posMin are 0.f, which means no limit
        float posMin = 0.f;
        float posMax = 0.f;
    };
};

struct InitConfig_s {
    uint32_t *pComHandle;
    ComType_e comType;
    WorkMode_e workMode;
    uint8_t offsetId;
    float txFreq;

    std::unique_ptr<PID> posPID;
    std::unique_ptr<PID> velPID;
    std::unique_ptr<PID> torqPID;

    bool isReverse = false;
};

struct Model_s {
    char name[12];        // 电机型号名
    uint16_t measureMin;  // 测量最小值
    uint16_t measureMax;  // 测量最大值
    float reductionRatio; // 减速比
    uint16_t txBaseId;    // 发送基ID
    uint16_t rxBaseId;    // 接收基ID
};

struct Data_s {
    float rawAng;  // 原生编码角度的映射 零点由电机内部编码器决定 映射为rad
    float zeroAng; // 零点角度值 rad
    float ang;     // 相对零点的角度 rad
    float angLast; // 上一时刻的相对零点的角度 rad

    float singleCirAng; // 相对零点单圈值 rad
    float multipCirAng; // 设置零点后清空 rad

    float cirNum; // 相对零点的圈数

    float spdRadps; // 角速度 rad/s
    float spdRpm;   // 角速度 rpm
    float curr;     // 电流   A
    float torq;     // 扭矩   Nm
    float tempture; // 温度   °C
};

struct Cmd_s {
    bool SW; // enable flag
    bool prevSW;
    MotorCmdType_e curCmdType = MotorCmdType_e::OFF;

    float velMax = -1.f; // rad/s
    float posMax = 0.f;  // rad
    float posMin = 0.f;  // rad
    struct {
        float pos;
        float vel;
        float torq;
        float elec; // common elecric input (current or voltage)
    };
    void clear()
    {
        SW = prevSW = false;
        torq = 0.f;
    }
    void updateSW(bool _sw)
    {
        if (_sw != prevSW) {
            SW = _sw;
            prevSW = _sw;
        }
    }
};

struct RxBus_s {
    template <uint8_t Len> struct CANRxBuf_s {
        uint8_t data[Len];
        const uint8_t len = Len;
    };
};

struct TxBus_s {
    template <uint8_t Len> struct CANTxBuf_s {
        uint8_t data[Len];
        uint8_t len = Len;
    };
};
} // namespace PINYMOTOR
