#pragma once

#include "IMotor.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <unordered_map>
#include <cstring>
#include <functional>

namespace PINYMOTOR {

// 模板类，用于定义一拖一电机的基类
template <typename Derived> class MotorBase : public IMotor {
private:
    inline Derived &derived() { return static_cast<Derived &>(*this); }
    inline const Derived &derived() const { return static_cast<const Derived &>(*this); }
protected:

    uint32_t *pComHandle_;
    ComType_e comType_;
    WorkMode_e workMode_;
    GlobalState_e globalState_;
    uint8_t offsetId_;

    // 用户自定义回调函数
    std::function<void(const uint8_t *_rxBuffer)> userRecvCallback_;


    bool isQuad_ = false; // default is not quad encoder

    uint32_t lastSendTick = 0; // ms

public:
    inline MotorBase(const char _name[16], InitConfig_s _config)
            : pComHandle_(_config.pComHandle)
            , comType_(_config.comType)
            , workMode_(_config.workMode)
            , globalState_(GlobalState_e::UNREGISTER)
            , offsetId_(_config.offsetId)
    {
        this->txFreq_ = _config.txFreq;

        strcpy(this->name_, _name);

        checkConfig();
    }

    inline MotorTypeDef_e checkConfig() { return derived()._checkConfig_(); }
    
    inline MotorTypeDef_e send(uint8_t *_txBuffer,
                               uint8_t _txLen) override final
    {
        return derived()._send_(_txBuffer, _txLen);
    }

    inline MotorTypeDef_e parse(const uint8_t *_rxBuffer) override final
    {
        return derived()._parse_(_rxBuffer);
    }

    inline MotorTypeDef_e ctrl() override final { return derived()._ctrl_(); }

    inline MotorTypeDef_e cmd(MotorCmdType_e _cmd, float _cmdData) override final
    {
        return derived()._cmd_(_cmd, _cmdData);
    }
    inline MotorTypeDef_e cmd(MotorCmdType_e _cmd) override final
    {
        return derived()._cmd_(_cmd);
    }

    inline void
    regUserRecvCallback(std::function<void(const uint8_t *_rxBuf)> _callback)
    {
        userRecvCallback_ = std::move(_callback);
    }

    inline uint16_t canId() { return derived().canId(); }

    inline uint16_t masterId() { return derived().masterId(); }

    inline uint16_t uid() override final { return derived()._uid_();}

    inline bool checkSend() const
    {
        return (xTaskGetTickCount() - lastSendTick) >=
               pdMS_TO_TICKS(1000.f / this->txFreq());
    }
};

/*******************************************************************/
struct QuadMotorGroup_s {
    IMotor *motor[4];
    uint8_t package[8];
    uint32_t lastSendTick; // ms
    float minTxFreq;
    QuadMotorGroup_s()
    {
        for (int i = 0; i < 4; i++)
            motor[i] = nullptr;
        memset(package, 0, sizeof(package));
        lastSendTick = 0.f;
        minTxFreq = 1000.f; // 初始设成最大，以便后续更新减小
    }
    void showMotorInfo()
    {
        for (int i = 0; i < 4; i++) {
            if (motor[i] != nullptr) {
                motor[i]->log("INFO", "green", "Motor %s: exist, uid: %hx",
                          motor[i]->getName(), motor[i]->uid());
            } else {
                motor[i]->log("INFO", "red", "Motor %d: not exist", i);
            }
        }
    }
};
// 模板类，用于定义一拖四电机的基类
template <typename Derived> class QuadMotorBase : public MotorBase<Derived> {
    // 使用基类的成员变量和成员函数
    using Base = MotorBase<Derived>;
protected:
    // vector < pair(pComHandle_, <canId, 4 motors>) >
    inline static std::vector<std::pair<
            uint32_t *, std::unordered_map<uint16_t, QuadMotorGroup_s *> > >
            motorMap_;

    inline void updateMotorMap()
    {
        // 注册电机到motorMap_中
        // 先寻找是否存在对应的pComHandle_
        auto it = std::ranges::find_if(motorMap_.begin(), motorMap_.end(),
                                [this](const auto &pair) {
                                    return pair.first == this->pComHandle_;
                                }); // lamda
        if (it == motorMap_.end()) {
            // 如果不存在，则直接在motorMap_尾部增多一个pair对象
            motorMap_.emplace_back(
                    this->pComHandle_,
                    std::unordered_map<uint16_t, QuadMotorGroup_s *>());
            it = motorMap_.end() - 1;
        }
        // 在找到的pair对象中添加电机组
        auto &map = it->second;
        // 检查pair中是否已经存在电机组
        if (map.find(getGroupId()) == map.end()) {
            // 如果不存在，则创建一个电机组
            map[getGroupId()] = new QuadMotorGroup_s();
            map[getGroupId()]->motor[getPosInGroup()] = this;
            this->log("INFO", "green", "Motor %s: create QuadMotorGroup %hx",
                      this->name_, getGroupId());
        } else {
            // 如果存在，则检查电机组中是否已经存在该电机
            if (map[getGroupId()]->motor[getPosInGroup()] != nullptr) {
                this->log("ERROR", "red", "Motor %s: already exist",
                          this->name_);
                return;
            } else {
                map[getGroupId()]->motor[getPosInGroup()] = this;
            }
        }
        // 检查电机组中所有电机的发送频率是否一致，并更新最小发送频率
        for (size_t i = 0; i < 4; i++) {
            if (map[getGroupId()]->motor[i] != nullptr) {
                if (map[getGroupId()]->motor[i]->txFreq() != this->txFreq()) {
                    this->log("WARN", "", "Motor %s: txFreq not match",
                              this->name_);
                }
            }
            map[getGroupId()]->minTxFreq =
                    std::min(map[getGroupId()]->minTxFreq, this->txFreq());
        }

        map[getGroupId()]->showMotorInfo();
    }

public:
    // 构造函数，初始化基类
    QuadMotorBase(const char _name[16], InitConfig_s _config) : Base(_name, _config)
    {
        
    }
    
    // 获取电机的组ID
    inline uint16_t getGroupId() const { return this->model_.txBaseId; }

    // 获取电机在组中的位置
    inline uint8_t getPosInGroup() const
    {
        return this->offsetId_ % 4; // 0,1,2,3
    }

    inline bool checkGroupSend(QuadMotorGroup_s *_group) const
    {
        return (xTaskGetTickCount() - _group->lastSendTick) >=
               pdMS_TO_TICKS(1000.f / _group->minTxFreq);
    }
};

/*******************************************************************/
struct TripMotorGroup_s {
    IMotor *motor[3];
    uint8_t package[8];
    uint32_t lastSendTick; // ms
    float minTxFreq;
    TripMotorGroup_s()
    {
        for (int i = 0; i < 3; i++)
            motor[i] = nullptr;
        memset(package, 0, sizeof(package));
        lastSendTick = 0.f;
        minTxFreq = 1000.f; // 初始设成最大，以便后续更新减小
    }
    void showMotorInfo()
    {
        for (int i = 0; i < 3; i++) {
            if (motor[i] != nullptr) {
                motor[i]->log("INFO", "green", "Motor %s: exist, uid: %hx",
                          motor[i]->getName(), motor[i]->uid());
            } else {
                motor[i]->log("INFO", "red", "Motor %d: not exist", i);
            }
        }
    }
};
// 模板类，用于定义一拖三电机的基类
template <typename Derived> class TripMotorBase : public MotorBase<Derived> {
    using Base = MotorBase<Derived>;
protected:
    // vector < pair(pComHandle_, <canId, 3 motors>) >
    inline static std::vector<std::pair<
            uint32_t *, std::unordered_map<uint16_t, TripMotorGroup_s *> > >
            motorMap_;

    inline void updateMotorMap()
    {
        auto it = std::ranges::find_if(motorMap_.begin(), motorMap_.end(),
                                [this](const auto &pair) {
                                    return pair.first == this->pComHandle_;
                                }); // lamda
        if (it == motorMap_.end()) {
            motorMap_.emplace_back(
                    this->pComHandle_,
                    std::unordered_map<uint16_t, TripMotorGroup_s *>());
            it = motorMap_.end() - 1;
        }
        auto &map = it->second;
        if (map.find(getGroupId()) == map.end()) {
            map[getGroupId()] = new TripMotorGroup_s();
            map[getGroupId()]->motor[getPosInGroup()] = this;
            this->log("INFO", "green", "Motor %s: create TripMotorGroup %hx",
                      this->name_, getGroupId());
        } else {
            if (map[getGroupId()]->motor[getPosInGroup()] != nullptr) {
                this->log("ERROR", "red", "Motor %s: already exist",
                          this->name_);
                return;
            } else {
                map[getGroupId()]->motor[getPosInGroup()] = this;
            }
        }
        for (size_t i = 0; i < 3; i++) {
            if (map[getGroupId()]->motor[i] != nullptr) {
                if (map[getGroupId()]->motor[i]->txFreq() != this->txFreq()) {
                    this->log("WARN", "", "Motor %s: txFreq not match",
                              this->name_);
                }
            }
            map[getGroupId()]->minTxFreq =
                    std::min(map[getGroupId()]->minTxFreq, this->txFreq());
        }

        map[getGroupId()]->showMotorInfo();
    }

public:
    TripMotorBase(const char _name[16], InitConfig_s _config) : Base(_name, _config)
    {

    }
    inline uint16_t getGroupId() const { return this->model_.txBaseId; }
    inline uint8_t getPosInGroup() const
    {
        return this->offsetId_ % 3; // 0,1,2
    }
    inline bool checkGroupSend(TripMotorGroup_s *_group) const
    {
        return (xTaskGetTickCount() - _group->lastSendTick) >=
               pdMS_TO_TICKS(1000.f / _group->minTxFreq);
    }
};
}
