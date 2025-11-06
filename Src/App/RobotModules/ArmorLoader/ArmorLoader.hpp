#pragma once

#include "FSMState.hpp"
#include "MsgImpl.hpp"

namespace ARMORLOADER {
enum class FSMState_e : uint8_t { STOP = 1, SINGLE, AUTO, BLOCK };
}

class ArmorLoader {
public:
    ArmorLoaderMsg_s msg = {};

protected:
    StateFactory<ARMORLOADER::FSMState_e> stateFactory_;
};
