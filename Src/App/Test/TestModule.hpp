#pragma once

#include "FSMState.hpp"
#include "StmLog.hpp"
#include "Singleton.hpp"

#include "Task.hpp"

namespace TEST {
enum class FSMState_e : uint8_t {};
}

class TestModule : public Singleton<TestModule>, public Task<TestModule, 256> {
public:
    void init();

    void update();

    void task();

    StateFactory<TEST::FSMState_e> stateFactory_;

    /*MOTOR*/

private:
    TestModule() : Task("TestTask", TaskPriority_e::HIGH1) {};
    friend class Singleton<TestModule>;
};
