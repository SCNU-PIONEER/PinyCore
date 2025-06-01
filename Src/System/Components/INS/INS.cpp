#include "INS.hpp"

#include <cstring>


void INS::update(IMUSensorData_s *_sensorDat, float _dt)
{
    memcpy(&this->rawDat, _sensorDat, sizeof(IMUSensorData_s));
}

