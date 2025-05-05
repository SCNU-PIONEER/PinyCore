#include "M3508.hpp"

#include "DJIMotor.hpp"

#include <cstring>

using namespace PINYMOTOR;
M3508::M3508(const char _name[16], InitConfig_s _config)
        : DJIMotor<M3508>(_name, _config)
{
    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = _config.reductionRatio;
    this->model_.rxBaseId = 0x200;
    if(_config.offsetId > 3)
        this->model_.txBaseId = 0x1FF;
    else
        this->model_.txBaseId = 0x200;
    
    this->stats_ = DJIMotorStats_s(
        25000.f, // voltTxCodeSpan
        16384.f, // currTxCodeSpan
        8192.f,  // currRxCodeSpan
                                        
        10.f, // currRated
        3.f,  // torqRated
         
        25.2f, // voltMax
        2.5f,  // currMax
        4.5f, // torqMax
                              
        0.3f // torqConstant
    );
    this->log("INFO", "",
                          "Motor %s: An instance of DJIMotor created", this->name_);
}