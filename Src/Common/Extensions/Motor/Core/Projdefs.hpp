#pragma once
#include <cstdint>

#include "pid.hpp"
namespace PINYMOTOR {

using MotorTypeDef_e = uint8_t;

enum class MotorCmdType_e { SET_VEL, SET_POS, SET_TORQ, SET_ELEC, ON, OFF };

enum class WorkMode_e {
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
enum class ComType_e { NONE, FDCAN, CAN, RS485 };

enum class GlobalState_e { UNREGISTER, OFFLINE, ONLINE, ERROR };

enum class MotorErrorCode_e { ErrorNone = 0u };
struct InitConfig_s {
    uint32_t *pComHandle;
    ComType_e comType;
    WorkMode_e workMode;
    uint8_t offsetId;
    float txFreq;

    PID *posPID;
    PID *velPID;
    PID *torqPID;
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
    int32_t rawScale;     // 原始比例
    int32_t lastRawScale; // 上一次原始比例
    float singleCirAng;   // rad
    float multipCirAng;   // rad
    float spdRadps;       // rad/s
    float spdRpm;         // rpm
    float curr;           // A
    float torq;           // Nm
    float tempture;       // °C
    float feedbackFreq;   // Hz
};

struct Cmd_s {
    bool SW; // enable flag
    bool prevSW;
    struct {
        float pos;
        float vel;
        float torq;
        float elec; // common elecric input (current or voltage)
    };
    inline void clear()
    {
        SW = prevSW = false;
        torq = 0.f;
    }
    inline void updateSW(bool _sw)
    {
        if (_sw != prevSW) {
            SW = _sw;
            prevSW = _sw;
        }
    }
};

}