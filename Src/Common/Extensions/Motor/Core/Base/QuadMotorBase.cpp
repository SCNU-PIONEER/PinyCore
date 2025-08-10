#include "./MotorBase.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;

QuadMotorGroup_s::QuadMotorGroup_s()
        : refLoadedCode((0 << 0) | (0 << 1) | (0 << 2) | (0 << 3))
        , curLoadedCode((0 << 0) | (0 << 1) | (0 << 2) | (0 << 3))
        , lastSendTick(0.f)
        , minTxFreq(1000.f)
{
    for (auto &i : motor)
        i = nullptr;
    memset(txBuf.data, 0, 8);
}
void QuadMotorGroup_s::showMotorInfo()
{
    for (auto &i : motor) {
        if (i != nullptr) {
            LOG::info("QuadMotorGroup", "Motor %s: exist, uid: %hx",
                      i->getName(), i->uid());
        } else {
            LOG::warn("QuadMotorGroup", "Motor %d: not exist", i);
        }
    }
}
QuadMotorBase ::QuadMotorBase(const char _name[16], InitConfig_s _config)
        : Base(_name, std::move(_config))
{
    isMutiple_ = true;
}

QuadMotorBase::QuadMotors &QuadMotorBase::getMotorMap()
{
    static QuadMotors motorMap;
    return motorMap;
}

void QuadMotorBase::updateMotorMap()
{
    // 注册电机到motorMap_中
    // 先寻找是否存在对应的pComHandle_
    auto it = getMotorMap().end();
    for (auto iter = getMotorMap().begin(); iter != getMotorMap().end();
         ++iter) {
        if (iter->handle == this->pComHandle_) {
            it = iter;
            break;
        }
    }
    if (it == getMotorMap().end()) {
        // 如果不存在，则直接在motorMap_尾部增多一个pair对象
        getMotorMap().emplace_back(
                this->pComHandle_,
                std::unordered_map<uint16_t, QuadMotorGroup_s *>());
        it = getMotorMap().end() - 1;
    }

    // 在找到的pair对象中添加电机组
    auto &map = it->groupMap;

    // 检查pair中是否已经存在电机组
    if (map.find(getGroupId()) == map.end()) {
        // 如果不存在，则创建一个电机组
        map[getGroupId()] = new QuadMotorGroup_s();
        map[getGroupId()]->motor[getPosInGroup()] = this;
        map[getGroupId()]->refLoadedCode |= (1 << getPosInGroup());
        LOG::info("QuadMotorBase", "Motor %s: create QuadMotorGroup %hx",
                  this->name_, getGroupId());
    } else {
        // 如果存在，则检查电机组中是否已经存在该电机
        if (map[getGroupId()]->motor[getPosInGroup()] != nullptr) {
            LOG::error("QuadMotorBase",
                       "Motor %s: already exist in group %hx, pos in group: %d",
                       this->name_, getGroupId(), getPosInGroup());
            return;
        } else {
            map[getGroupId()]->motor[getPosInGroup()] = this;
            map[getGroupId()]->refLoadedCode |= (1 << getPosInGroup());
        }
    }
    group_ = map[getGroupId()];

    // 检查电机组中所有电机的发送频率是否一致，并更新最小发送频率
    for (auto &i : map[getGroupId()]->motor) {
        if (i != nullptr) {
            if (i->txFreq() != this->txFreq()) {
                LOG::warn("QuadMotorBase", "Motor %s: txFreq not match",
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
    auto it = getMotorMap().end();
    for (auto iter = getMotorMap().begin(); iter != getMotorMap().end();
         ++iter) {
        if (iter->handle == this->pComHandle_) {
            it = iter;
            break;
        }
    }
    if (it != getMotorMap().end()) {
        auto &map = it->groupMap;
        if (map.find(getGroupId()) != map.end()) {
            map[getGroupId()]->motor[getPosInGroup()] = nullptr;
            map[getGroupId()]->refLoadedCode &= ~(1 << getPosInGroup());
            LOG::info("QuadMotorBase",
                      "Motor %s: remove from group %hx, pos in group: %d",
                      this->name_, getGroupId(), getPosInGroup());
        } else {
            LOG::error("QuadMotorBase", "Motor %s: not in group %hx",
                       this->name_, getGroupId());
        }
    } else {
        LOG::error("QuadMotorBase", "Motor %s: not in motorMap_", this->name_);
    }
}

uint16_t QuadMotorBase::getGroupId() const { return this->model_.txBaseId; }

uint8_t QuadMotorBase::getPosInGroup() const
{
    return (this->offsetId_ - 1) % 4; // 0,1,2,3
}

bool QuadMotorBase::checkGroupSend(QuadMotorGroup_s *_group)
{
    if ((xTaskGetTickCount() - _group->lastSendTick) >=
                pdMS_TO_TICKS(1000.f / _group->minTxFreq) &&
        (_group->refLoadedCode == _group->curLoadedCode)) {
        _group->lastSendTick = xTaskGetTickCount();
        _group->curLoadedCode = 0; // reset curLoadedCode
        return true;
    } else {
        _group->curLoadedCode |= (1 << getPosInGroup());
        return false;
    }
}
