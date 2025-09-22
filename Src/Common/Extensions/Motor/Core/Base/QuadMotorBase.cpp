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
    memset(txBuf, 0, 8);
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
        : Base(_name, _config)
{
    regInfo_.isMutiple = true;
}

QuadMotorBase::QuadMotors &QuadMotorBase::getMotorMap()
{
    static QuadMotors motorMap;
    return motorMap;
}

void QuadMotorBase::updateMotorMap()
{
    // first, find the pair object in motorMap_ that contains the handle
    auto it = getMotorMap().end();
    for (auto iter = getMotorMap().begin(); iter != getMotorMap().end();
         ++iter) {
        if (iter->handle == regInfo_.pComHandle) {
            it = iter;
            break;
        }
    }
    if (it == getMotorMap().end()) {
        // if not found, create a new pair object
        getMotorMap().emplace_back(
                regInfo_.pComHandle,
                std::unordered_map<uint16_t, QuadMotorGroup_s *>());
        it = getMotorMap().end() - 1;
    }

    // add the motor to the groupMap_ of the pair object
    auto &map = it->groupMap;

    // check if the motor is already in the groupMap_
    if (!map.contains(getGroupId())) {
        // if not, create a new groupMap_
        map[getGroupId()] = new QuadMotorGroup_s();
        map[getGroupId()]->motor[getPosInGroup()] = this;
        map[getGroupId()]->refLoadedCode |= (1 << getPosInGroup());
        LOG::info("QuadMotorBase", "Motor %s: create QuadMotorGroup %hx",
                  regInfo_.name, getGroupId());
    } else {
        // if already exist, check if the motor is already in the groupMap_
        if (map[getGroupId()]->motor[getPosInGroup()] != nullptr) {
            LOG::error("QuadMotorBase",
                       "Motor %s: already exist in group %hx, pos in group: %d",
                       regInfo_.name, getGroupId(), getPosInGroup());
            return;
        } else {
            map[getGroupId()]->motor[getPosInGroup()] = this;
            map[getGroupId()]->refLoadedCode |= (1 << getPosInGroup());
        }
    }
    group_ = map[getGroupId()];

    // check if the txFreq is the same as the other motors in the group
    for (auto &i : map[getGroupId()]->motor) {
        if (i != nullptr) {
            if (i->txFreq() != this->txFreq()) {
                LOG::warn("QuadMotorBase", "Motor %s: txFreq not match",
                          regInfo_.name);
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
        if (iter->handle == regInfo_.pComHandle) {
            it = iter;
            break;
        }
    }
    if (it != getMotorMap().end()) {
        auto &map = it->groupMap;
        if (map.contains(getGroupId())) {
            map[getGroupId()]->motor[getPosInGroup()] = nullptr;
            map[getGroupId()]->refLoadedCode &= ~(1 << getPosInGroup());
            LOG::info("QuadMotorBase",
                      "Motor %s: remove from group %hx, pos in group: %d",
                      regInfo_.name, getGroupId(), getPosInGroup());
        } else {
            LOG::error("QuadMotorBase", "Motor %s: not in group %hx",
                       regInfo_.name, getGroupId());
        }
    } else {
        LOG::error("QuadMotorBase", "Motor %s: not in motorMap_",
                   regInfo_.name);
    }
}

uint16_t QuadMotorBase::getGroupId() const { return regInfo_.model.txBaseId; }

uint8_t QuadMotorBase::getPosInGroup() const
{
    return (regInfo_.offsetId - 1) % 4; // 0,1,2,3
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
