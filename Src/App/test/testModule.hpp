#include "FSMState.hpp"

#include "DM4310.hpp"

class TestModule {
public:
    inline static TestModule *instance() {
        static TestModule instance_;
        return &instance_;
    }

    TestModule(const TestModule &) = delete;

    void init();
    void task();

    auto motorTestTask() -> void;

private:
    TestModule() = default;

    /*MOTOR*/
    PINYMOTOR::DM4310 *testMotor_;
};