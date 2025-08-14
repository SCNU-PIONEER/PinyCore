#pragma once

#include "IMotor.hpp"

#include "INS.hpp"
#include "Topic.hpp"


namespace GIMBAL {

class Head {
public:
    virtual void stop() = 0;
    virtual void enter() = 0;
    virtual void update() = 0;
    virtual PINYMOTOR::IMotor *joint(uint8_t _jId) const = 0;
    virtual ~Head() = default;

    const INS_SYS::INSData_s &ins() const { return insData_; }

protected:
    INS_SYS::INSData_s insData_ = {};
    Subscriber<INS_SYS::INSData_s> *insSub_;
};
} // namespace GIMBAL
