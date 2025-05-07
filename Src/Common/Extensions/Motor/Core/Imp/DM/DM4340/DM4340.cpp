#include "DM4340.hpp"

#include "DMMotor.hpp"
#include <cstring>

using namespace PINYMOTOR;
DM4340::DM4340(const char _name[16], InitConfig_s _config)
        : DMMotor<DM4340>(_name, _config)
{
    strcpy(this->model_.name, "DM-DM4340");
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
        9.f,  // torqRated

        8.f, // currMax
        27.f,  // torqMax

        0.9333f // torqConstant
    };
    this->log("INFO", "",
                          "Motor %s: An instance of DM4310 created", this->name_);
    // TODO:
}