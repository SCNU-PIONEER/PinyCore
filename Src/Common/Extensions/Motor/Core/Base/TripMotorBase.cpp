#include "./MotorBase.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;

TripMotorGroup_s::TripMotorGroup_s()
        : refLoadedCode((0 << 0) | (0 << 1) | (0 << 2))
        , curLoadedCode((0 << 0) | (0 << 1) | (0 << 2))
        , lastSendTick(0.f)
        , minTxFreq(1000.f)
{
    for (auto &i : motor)
        i = nullptr;
    memset(txBuf, 0, 8);
}
void TripMotorGroup_s::showMotorInfo()
{
    for (auto &i : motor) {
        if (i != nullptr) {
            LOG::info("TripMotorGroup", "Motor %s: exist, uid: %hx",
                      i->getName(), i->uid());
        } else {
            LOG::warn("TripMotorGroup", "Motor %d: not exist", i);
        }
    }
}

TripMotorBase::TripMotorBase(const char _name[16], InitConfig_s _config)
        : Base(_name, std::move(_config))
{
    isMutiple_ = true;
}

TripMotorBase::TripMotors &TripMotorBase::getMotorMap()
{
    static TripMotors motorMap;
    return motorMap;
}

void TripMotorBase::updateMotorMap()
{
    auto it = getMotorMap().end();
    for (auto iter = getMotorMap().begin(); iter != getMotorMap().end();
         ++iter) {
        if (iter->handle == this->pComHandle_) {
            it = iter;
            break;
        }
    }
    if (it == getMotorMap().end()) {
        getMotorMap().emplace_back(
                this->pComHandle_,
                std::unordered_map<uint16_t, TripMotorGroup_s *>());
        it = getMotorMap().end() - 1;
    }

    auto &map = it->groupMap;

    if (map.find(getGroupId()) == map.end()) {
        map[getGroupId()] = new TripMotorGroup_s();
        map[getGroupId()]->motor[getPosInGroup()] = this;
        map[getGroupId()]->refLoadedCode |= (1 << getPosInGroup());
        LOG::info("TripMotorBase",
                  "Motor %s: add to group %hx, pos in group: %d", this->name_,
                  getGroupId(), getPosInGroup());
    } else {
        if (map[getGroupId()]->motor[getPosInGroup()] != nullptr) {
            LOG::error("TripMotorBase",
                       "Motor %s: already exist in group %hx, pos in group: %d",
                       this->name_, getGroupId(), getPosInGroup());
            return;
        } else {
            map[getGroupId()]->motor[getPosInGroup()] = this;
            map[getGroupId()]->refLoadedCode |= (1 << getPosInGroup());
        }
    }
    group_ = map[getGroupId()];

    for (auto &i : map[getGroupId()]->motor) {
        if (i != nullptr) {
            if (i->txFreq() != this->txFreq()) {
                LOG::warn("TripMotorBase", "Motor %s: txFreq not match",
                          this->name_);
            }
        }
        map[getGroupId()]->minTxFreq =
                std::min(map[getGroupId()]->minTxFreq, this->txFreq());
    }

    map[getGroupId()]->showMotorInfo();
}

void TripMotorBase::removeMotorFromMap()
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
            LOG::info("TripMotorBase", "Motor %s: remove from group %hx",
                      this->name_, getGroupId());
        } else {
            LOG::error("TripMotorBase", "Motor %s: not in group %hx",
                       this->name_, getGroupId());
        }
    } else {
        LOG::error("TripMotorBase", "Motor %s: not in motorMap_", this->name_);
    }
}

uint16_t TripMotorBase::getGroupId() const { return this->model_.txBaseId; }

uint8_t TripMotorBase::getPosInGroup() const
{
    return (this->offsetId_ - 1) % 3; // 0,1,2
}

bool TripMotorBase::checkGroupSend(TripMotorGroup_s *_group)
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
