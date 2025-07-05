// #include "FSMState.hpp"

#include "DM4310.hpp"

#include "GM3510.hpp"

#include "GM6020.hpp"

class TestModule {
public:
    inline static TestModule *instance() {
        static TestModule instance_;
        return &instance_;
    }

    TestModule(const TestModule &) = delete;

    void init();

    void task();
    void taskCreate();

    uint32_t debugCnt = 0;

private:
    TestModule() = default;

    /*MOTOR*/
    PINYMOTOR::DM4310 *testDM4310Motor_;
    PINYMOTOR::GM3510 *testGM3510Motor_;
    PINYMOTOR::GM6020 *testGM6020Motor_;

    auto motorTestTask() -> void;
};
