#include "ArmKinematic.hpp"
#include "Pump.hpp"

Joint7D operator-(const Joint7D &_joints1, const Joint7D &_joints2)
{
    Joint7D tmp{};
    for (int i = 0; i < 6; i++)
        tmp.j[i] = _joints1.j[i] - _joints2.j[i];
    return tmp;
}


const Joint7D silverLeftStorage[7] = {
    { 0, 0.70, -0.98, 0.99, -1.71, -1.82, 0, 500 },
    { 0, 0.83, -0.173, 0.99, -1.57, -1.65, 0, 1000 },
    { 0, 0.64, -0.85, 0.99, -1.57, -1.65, 0, 200 },
    { 0, 0.595, -1.2, 0.99, -1.57, -1.65, 0, 0 },
    { -2.86, 0.59, -1.2, 0.38, -1.57, -1.65, 0, 0 },
    { -2.86, 0.668, -0.88, 0.38, -0.23, 0, 0, 100 },
    { -2.86, 0.668, -1.2, 0.38, -0.23, 0, 0, 0 }
};


const JointRoute_s silverLeftRoute = { .pose = silverLeftStorage,
                                       .point = 7,
                                       .pump = PUMP::SILVER_PUMP_TEST };
