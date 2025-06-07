#include "./MotorBase.hpp"

using namespace PINYMOTOR;

TripMotorBase::TripMotors TripMotorBase::motorMap_ = {};

TripMotorGroup_s::TripMotorGroup_s()
{
    for (int i = 0; i < 3; i++)
        motor[i] = nullptr;
    memset(package, 0, sizeof(package));
    lastSendTick = 0.f;
    minTxFreq = 1000.f;
}
void TripMotorGroup_s::showMotorInfo()
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

TripMotorBase::TripMotorBase(const char _name[16], InitConfig_s _config) : Base(_name, _config)
{
    isMutiple_ = true;
}

TripMotorBase::TripMotors &TripMotorBase::getMotorMap() const
{
    return motorMap_;
}

void TripMotorBase::updateMotorMap()
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

void TripMotorBase::removeMotorFromMap()
{
    auto it = std::ranges::find_if(motorMap_.begin(), motorMap_.end(),
                            [this](const auto &pair) {
                                return pair.first == this->pComHandle_;
                            }); // lamda
    if (it != motorMap_.end()) {
        auto &map = it->second;
        if (map.find(getGroupId()) != map.end()) {
            map[getGroupId()]->motor[getPosInGroup()] = nullptr;
            this->log("INFO", "green", "Motor %s: remove from group %hx",
                      this->name_, getGroupId());
        } else {
            this->log("ERROR", "red", "Motor %s: not in group %hx",
                      this->name_, getGroupId());
        }
    } else {
        this->log("ERROR", "red", "Motor %s: not in motorMap_", this->name_);
    }
}

uint16_t TripMotorBase::getGroupId() const { return this->model_.txBaseId; }

uint8_t TripMotorBase::getPosInGroup() const
{
    return this->offsetId_ % 3; // 0,1,2
}

bool TripMotorBase::checkGroupSend(TripMotorGroup_s *_group)
{
    return (xTaskGetTickCount() - _group->lastSendTick) >=
           pdMS_TO_TICKS(1000.f / _group->minTxFreq);
}
