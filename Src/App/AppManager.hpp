#pragma once

#include "Task.hpp"
#include "etl/delegate.h"
#include <vector>
#include "Singleton.hpp"

class AppManager : public Singleton<AppManager>, public Task<AppManager, 512> {
public:
    void initApp();

    template <typename F> requires std::invocable<F> void schedule(F _callback) { tasks.push_back(_callback); }

    void task();

private:
    AppManager() : Task("AppTask", TaskPriority_e::HIGH1) {};
    friend class Singleton<AppManager>;

    void createApp();

    std::vector<etl::delegate<void()> > tasks;
};
