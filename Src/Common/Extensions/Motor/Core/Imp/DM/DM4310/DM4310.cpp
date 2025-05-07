#include "DM4310.hpp"

#include "DMMotor.hpp"
#include <cstring>

using namespace PINYMOTOR;
DM4310::DM4310(const char _name[16], InitConfig_s _config)
        : DMMotor<DM4310>(_name, _config)
{
    strcpy(this->model_.name, "DM-DM4310");
    this->model_.measureMax = 16383;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 1.f;
    this->model_.rxBaseId = 0x10;
    this->model_.txBaseId = 0x00;

    this->stats_ = DMMotorStats_s{
        12.5f, // PMax
        30.f,  // VMax
        10.f,  // TMax

        500.f, // MITKpMax
        5.f,   // MITKdMax

        10000.f, // currTxCodeSpan

        2.5f, // currRated
        3.f,  // torqRated

        7.5f, // currMax
        7.f,  // torqMax

        0.9333f // torqConstant
    };
    this->log("INFO", "",
                          "Motor %s: An instance of DM4310 created", this->name_);
    // TODO:
}
