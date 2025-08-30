#pragma once

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

private:
    AppManager() = default;
    void createApp();
};
