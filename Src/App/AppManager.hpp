#pragma once

#include <functional>
#include "Task.hpp"

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

    void schedule(std::function<void()> _callback);

    void task();

private:
    AppManager() : Task("AppTask", TaskPriority_e::HIGH1) {};
    void createApp();

    std::vector<std::function<void()> > tasks;
};
