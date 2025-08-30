#pragma once

#include "FSMState.hpp"
#include "StmLog.hpp"

#include "RcMsg.hpp"

namespace TEST {
enum class FSMState_e : uint8_t {};
}

class TestModule {
public:
    static TestModule *instance();

    TestModule(const TestModule &) = delete;

    void init();

    void update();

    void task();

    RC::RcRawMsg_t rcMsg;

    StateFactory<TEST::FSMState_e> stateFactory_;

    /*MOTOR*/

private:
    TestModule() = default;
};
