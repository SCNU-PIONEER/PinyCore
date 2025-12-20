#pragma once

#include "Task.hpp"
#include <initializer_list>
#include "etl/delegate.h"
#include <vector>

class AppManager : public Task<AppManager, 512> {
public:
    static AppManager *instance()
    {
        static AppManager instance;
        return &instance;
    }

    AppManager(const AppManager &) = delete;
    AppManager &operator=(const AppManager &) = delete;

    void initApp();

    template <typename F> requires std::invocable<F> void schedule(F _callback) { tasks.push_back(_callback); }

    void task();

private:
    AppManager() : Task("AppTask", TaskPriority_e::HIGH1) {};
    void createApp();

    std::vector<etl::delegate<void()> > tasks;
};
