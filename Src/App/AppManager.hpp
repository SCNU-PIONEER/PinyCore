#pragma once

#include <list>
#include <functional>

class AppManager {
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

    static void ctrlTask(void *_param);

private:
    AppManager() = default;
    void createApp();

    std::list<std::function<void()> > tasks;
};
