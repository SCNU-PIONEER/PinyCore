#pragma once

#include "FSMState.hpp"
#include "StmLog.hpp"

#include "Task.hpp"

namespace TEST {
enum class FSMState_e : uint8_t {};
}

class TestModule : public Task<TestModule, 256> {
public:
    static TestModule *instance();

    TestModule(const TestModule &) = delete;

    void init();

    void update();

    void task();

    StateFactory<TEST::FSMState_e> stateFactory_;

    /*MOTOR*/

private:
    TestModule() : Task("TestTask", TaskPriority_e::HIGH1) {};
};
