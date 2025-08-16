#include "Pump.hpp"
#include "StmLog.hpp"
// 实现初始化函数（仅在cpp中定义一次）
void PumpController::init()
{
    apply(PUMPCONFIGS::ALL_PUMP_OFF);
    LOG::Logger::instance().info(LOCATION, "Pump", "initialized");
}
