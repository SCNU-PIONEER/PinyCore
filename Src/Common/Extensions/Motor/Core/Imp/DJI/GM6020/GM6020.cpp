#include "GM6020.hpp"

#include "DJIMotor.hpp"

#include <cstring>

using namespace PINYMOTOR;
GM6020::GM6020(const char _name[16], InitConfig_s _config)
        : DJIMotor<GM6020>(_name, _config)
{
    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 1.f;
    this->model_.rxBaseId = 0x204;
    switch (_config.workMode)
    {
    case WorkMode_e::QUAD_CURR:{
        if(_config.offsetId > 3)
        this->model_.txBaseId = 0x2FE;
        else
        this->model_.txBaseId = 0x1FE;
    break;
    }
    case WorkMode_e::QUAD_VOLT:{
        if(_config.offsetId > 3)
        this->model_.txBaseId = 0x2FF;
        else
        this->model_.txBaseId = 0x1FF;
    break;
    }
    default:
    break;
    }
    this->log("WARNING", "", "Please check the WorkMode of %s", this->name_);

    this->stats_ = DJIMotorStats_s(
        25000.f, // voltTxCodeSpan
        16384.f, // currTxCodeSpan
        8192.f,  // currRxCodeSpan
                                        
        1.62f, // currRated
        1.2f,  // torqRated
         
        25.2f,  // voltMax
        0.9f,  // currMax
        0.86f, // torqMax
                              
        0.741f // torqConstant
    );
    this->log("INFO", "",
                          "Motor %s: An instance of DJIMotor created", this->name_);
}
