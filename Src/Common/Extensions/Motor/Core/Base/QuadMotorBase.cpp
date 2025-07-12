#include "./MotorBase.hpp"

using namespace PINYMOTOR;

QuadMotorBase::QuadMotors QuadMotorBase::motorMap_ = {};

QuadMotorGroup_s::QuadMotorGroup_s()
{
    for (int i = 0; i < 4; i++)
        motor[i] = nullptr;
    lastSendTick = 0.f;
    minTxFreq = 1000.f;
}
void QuadMotorGroup_s::showMotorInfo()
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
QuadMotorBase ::QuadMotorBase(const char _name[16], InitConfig_s _config)
        : Base(_name, _config)
{
    isMutiple_ = true;
}

QuadMotorBase::QuadMotors &QuadMotorBase::getMotorMap() const
{
    return motorMap_;
}

void QuadMotorBase::updateMotorMap()
{
    // 注册电机到motorMap_中
    // 先寻找是否存在对应的pComHandle_
    auto it = motorMap_.end();
    for (auto iter = motorMap_.begin(); iter != motorMap_.end(); ++iter) {
        if (iter->first == this->pComHandle_) {
            it = iter;
            break;
        }
    }
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
            this->log("ERROR", "red", "Motor %s: already exist", this->name_);
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

void QuadMotorBase::removeMotorFromMap()
{
    auto it = motorMap_.end();
    for (auto iter = motorMap_.begin(); iter != motorMap_.end(); ++iter) {
        if (iter->first == this->pComHandle_) {
            it = iter;
            break;
        }
    }
    if (it != motorMap_.end()) {
        auto &map = it->second;
        if (map.find(getGroupId()) != map.end()) {
            map[getGroupId()]->motor[getPosInGroup()] = nullptr;
            this->log("INFO", "green", "Motor %s: remove from group %hx",
                      this->name_, getGroupId());
        } else {
            this->log("ERROR", "red", "Motor %s: not in group %hx", this->name_,
                      getGroupId());
        }
    } else {
        this->log("ERROR", "red", "Motor %s: not in motorMap_", this->name_);
    }
}

uint16_t QuadMotorBase::getGroupId() const { return this->model_.txBaseId; }

uint8_t QuadMotorBase::getPosInGroup() const
{
    return (this->offsetId_ - 1) % 4; // 0,1,2,3
}

bool QuadMotorBase::checkGroupSend(QuadMotorGroup_s *_group)
{
    return (xTaskGetTickCount() - _group->lastSendTick) >=
           pdMS_TO_TICKS(1000.f / _group->minTxFreq);
}
