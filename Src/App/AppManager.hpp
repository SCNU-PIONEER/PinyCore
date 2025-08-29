#pragma once

class AppManager {
public:
    static AppManager *instance()
    {
        static AppManager instance_;
        return &instance_;
    }

    AppManager(const AppManager &) = delete;
    AppManager &operator=(const AppManager &) = delete;

    void initApp();

private:
    AppManager() = default;
    void createApp();
};
