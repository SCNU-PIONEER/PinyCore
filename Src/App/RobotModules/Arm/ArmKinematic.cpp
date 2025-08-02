#include "ArmKinematic.hpp"

Joint7D operator-(const Joint7D &_joints1, const Joint7D &_joints2)
{
    Joint7D tmp{};
    for (int i = 0; i < 6; i++)
        tmp.j[i] = _joints1.j[i] - _joints2.j[i];
    return tmp;
}
