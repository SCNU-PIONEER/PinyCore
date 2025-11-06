#pragma once

#include "FSMState.hpp"

#include "MsgImpl.hpp"

namespace ARMORBOOSTER {
enum class FSMState_e : uint8_t { OFF = 1, ON };
}

class ArmorBooster {
public:
    ArmorBoosterMsg_s msg = {};

protected:
    StateFactory<ARMORBOOSTER::FSMState_e> stateFactory_;
};
