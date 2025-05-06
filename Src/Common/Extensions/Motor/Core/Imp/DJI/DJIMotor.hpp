#pragma once

#include "../../Base/MotorBase.hpp"

#include <unordered_map>

namespace PINYMOTOR {

#pragma pack(push, 1)
struct DJIMotorMsg_s {
    int16_t cmd[4];
};
struct DJIMotorFeedback_s {
    uint16_t rawScale;
    int16_t rawRpm;
    int16_t current;
    uint8_t temperature;
};
#pragma pack(pop)
struct DJIMotorStats_s {
    float voltTxCodeSpan;
    float currTxCodeSpan;
    float currRxCodeSpan;
    float currRated;    // A
    float torqRated;    // Nm
    float voltMax;      // V
    float currMax;      // A
    float torqMax;      // Nm
    float torqConstant; // Nm/A

    DJIMotorStats_s& operator=(const DJIMotorStats_s& _other) {
        if (this != &_other)
        {
            voltTxCodeSpan = _other.voltTxCodeSpan;
            currTxCodeSpan = _other.currTxCodeSpan;
            currRxCodeSpan = _other.currRxCodeSpan;
            currRated = _other.currRated;
            torqRated = _other.torqRated;
            voltMax = _other.voltMax;
            currMax = _other.currMax;
            torqMax = _other.torqMax;
            torqConstant = _other.torqConstant;
        }
        return *this;
    }
};

template <typename Derived> class DJIMotor : public QuadMotorBase<DJIMotor<Derived>>{
    using Base = QuadMotorBase<DJIMotor<Derived> >;

private:
    // 注册解析函数
    void registerRecvCallback();

protected:
    DJIMotorStats_s stats_;

    // 一拖四电机特有的全局包实例
    static std::unordered_map<int, DJIMotor *> motorMap;

    struct Cmd_s {
        bool SW;
        bool prevSW;
        struct {
            float torq;
            float volt;
            // float speed;
            // float pos;
        };
        void clear() {
            SW = false;
            torq = 0;
            // speed = 0;
            // pos = 0;
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
    inline DJIMotor(const char _name[16], InitConfig_s _config)
            : Base(_name, _config)
    {
        cmd_.clear();
        registerRecvCallback(); // 注册解析函数
    }

    MotorTypeDef_e _cmd_(MotorCmdType_e _cmd, float _cmdData);
    MotorTypeDef_e _cmd_(MotorCmdType_e _cmd);

    inline uint16_t canId() { return this->model_.txBaseId + 0u; } // QuadMotor's canId is fixed
    inline uint16_t masterId()
    {
        return this->model_.rxBaseId + this->offsetId_;
    }

    inline uint16_t _uid_() { return masterId(); }

    MotorTypeDef_e _send_(uint8_t *_txBuf, uint8_t _len);

    MotorTypeDef_e _parse_(uint8_t *_rxBuf);

    MotorTypeDef_e _ctrl_();
    
};
}