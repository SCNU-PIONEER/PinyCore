#pragma once

#include <vector>
#include "FSMState.hpp"
#include "Singleton.hpp"
#include "Task.hpp"
#include "etl/delegate.h"

namespace TEST {
enum class FSMState_e : uint8_t {};
}

class TestModule : public Singleton<TestModule>, public Task<TestModule, 512> {
public:
    void init();

    void update();

    void task();

    StateFactory<TEST::FSMState_e> stateFactory_;

    template <typename F> requires std::invocable<F> void schedule(F _callback) { tasks.push_back(_callback); }

    enum class DmMotorModel_e : uint8_t { DM4310, DM3507, DM3519, DM4340, DM6006, DM8009, DM10010L };
    /*MOTOR*/

protected:
    TestModule() : Task("TestTask", TaskPriority_e::HIGH1) {};
    friend class Singleton<TestModule>;
    std::vector<etl::delegate<void()> > tasks;
};
