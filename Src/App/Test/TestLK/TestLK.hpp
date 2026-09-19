#pragma once
#include "LKMotor.hpp"
#include "TestModule.hpp"
#include <cstdint>
namespace TEST {

enum class LkMotorModel_e : uint8_t { MG4005, MF9025 };

class TestLK : public TestModule {
public:
    void addMotor(LkMotorModel_e _model, uint8_t _id);

    void test();
    void testON();
    void testOFF();
    void testTorq(float _torq);
    void testTorq(uint8_t _index, float _torq);

    PINYMOTOR::LKMOTOR::LKMotor *getMotor(uint8_t _index);

private:
    static constexpr uint8_t MAX_MOTORS = 4;
    std::array<PINYMOTOR::LKMOTOR::LKMotor *, MAX_MOTORS> motors_{};
    uint8_t motorCount_{ 0 };
};

} // namespace TEST
