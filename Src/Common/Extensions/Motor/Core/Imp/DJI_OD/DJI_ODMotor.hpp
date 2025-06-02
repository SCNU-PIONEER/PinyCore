#pragma once

#include "../../Base/MotorBase.hpp"

#include <unordered_map>

namespace PINYMOTOR {

#pragma pack(push, 1)
struct DJI_ODMotorMsg_s {
    int16_t cmd[3];
};

struct DJI_ODMotorFeedback_s {
    uint16_t rawScale;
    int16_t rawTorq;
};
#pragma pack(pop)
struct DJI_ODMotorStats_s {
    float voltTxCodeSpan;
    float torqRxCodeSpan;
    float currRated;    // A
    float torqRated;    // Nm
    float voltMax;      // V
    float currMax;      // A
    float torqMax;      // Nm
    float torqConstant; // Nm/A

    DJI_ODMotorStats_s& operator=(const DJI_ODMotorStats_s& _other) {
        if (this != &_other)
        {
            voltTxCodeSpan = _other.voltTxCodeSpan;
            torqRxCodeSpan = _other.torqRxCodeSpan;
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

template <typename Derived>
class DJI_ODMotor : public TripMotorBase<DJI_ODMotor<Derived> > {
    using Base = TripMotorBase<DJI_ODMotor<Derived> >;
    inline Derived &derived() { return static_cast<Derived &>(*this); }
    inline const Derived &derived() const
    {
        return static_cast<const Derived &>(*this);
    }

private:
    

protected:
    DJI_ODMotorStats_s stats_;
    void registerRecvCallback();
    struct Cmd_s {
        bool SW;
        bool prevSW;
        struct {
            float volt;
        };
        void clear() {
            SW = false;
        }
        void updateSW(bool _sw)
        {
            if (_sw != prevSW) {
                SW = _sw;
                prevSW = _sw;
            }
        }
    } cmd_;

    uint16_t ctrlId_ = 0x00;

public:
    inline DJI_ODMotor(const char _name[16], InitConfig_s _config)
            : Base(_name, _config)
    {
        cmd_.clear();
    }

    inline void overrideStats(const DJI_ODMotorStats_s& _newStats)
    {
        stats_ = _newStats;
    }

    inline MotorTypeDef_e _checkConfig_(){ return derived().checkBaseConfig(); }

    MotorTypeDef_e _cmd_(MotorCmdType_e _cmd, float _cmdData);
    MotorTypeDef_e _cmd_(MotorCmdType_e _cmd);

    inline uint16_t canId() { return this->model_.txBaseId + 0u; } // QuadMotor's canId is fixed
    inline uint16_t masterId()
    {
        return this->model_.rxBaseId + this->offsetId_;
    }

    inline uint16_t _uid_() { return masterId(); }

    MotorTypeDef_e _send_(uint8_t *_txBuf, uint8_t _len);

    MotorTypeDef_e _parse_(const uint8_t *_rxBuf);

    MotorTypeDef_e _ctrl_();

    inline TripMotorGroup_s *findGroup() const
    {
        TripMotorGroup_s *group = nullptr;
        for (auto &entry : this->motorMap_) {
            if (entry.first == this->pComHandle_) {
                auto it = entry.second.find(this->getGroupId()); // it" is a map
                if (it != entry.second.end()) {
                    group = it->second;
                    break;
                }
            }
        }
        return group;
    }
};
};
