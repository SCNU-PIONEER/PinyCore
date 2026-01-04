#pragma once
#include "M3508.hpp"

namespace ARM {
    
class Lifting {
public:
    Lifting();
    void lift();
    void stop();
    void enable();

private:
    PINYMOTOR::DJIMOTOR::M3508 motor;
};
} // namespace ARM