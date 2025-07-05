#include "M2006.hpp"

#include "DJIMotor.hpp"

#include <cstring>

using namespace PINYMOTOR;
M2006::M2006(const char _name[16], InitConfig_s _config)
        : DJIMotor(_name, _config)
{
    strcpy(this->model_.name, "DJI-M2006");
    this->model_.measureMax = 8191;
    this->model_.measureMin = 0;
    this->model_.reductionRatio = 36.f;
    this->model_.rxBaseId = 0x200;

    if (_config.offsetId > 3)
        this->model_.txBaseId = 0x1FF;
    else
        this->model_.txBaseId = 0x200;

    this->stats_ = DJIMotorStats_s(
            M2006_VOLT_TX_CODE_SPAN, // voltTxCodeSpan
            M2006_CURR_TX_CODE_SPAN, // currTxCodeSpan
            M2006_CURR_RX_CODE_SPAN, // currRxCodeSpan
            M2006_CURR_RATED,        // currRated
            M2006_TORQ_RATED,        // torqRated
            M2006_VOLT_MAX,          //voltmax
            //搭配c610无堵转电流和堵转扭矩数据，所以还是用额定数据
            M2006_CURR_MAX,     // currMax
            M2006_TORQ_MAX,     // torqMax
            M2006_TORQ_CONSTANT // torqConstant
    );

    this->registerMotor();
    this->updateMotorMap();
    this->registerRecvCallback();
    this->updateCtrlId();

    checkBaseConfig();

    this->log("INFO", "green", "Motor %s: An instance of DJIMotor created",
              this->name_);
}

MotorTypeDef_e M2006::checkBaseConfig()
{
    MotorTypeDef_e rslt = 0;

    if (this->comType_ != ComType_e ::CAN) {
        rslt |= 1;
        this->log("ERROR", "red", "M2006 %s: only support CAN comtype",
                  this->name_);
    }

    if (this->workMode_ != WorkMode_e::QUAD_CURR) {
        rslt |= 1;
        this->log("ERROR", "red", "M2006 %s: WorkMode is not supported",
                  this->name_);
    }

    if (this->offsetId_ > 8) {
        rslt |= 1;
        this->log("ERROR", "red", "M2006 %s: Max Offset ID is only 8!",
                  this->name_);
    }

    if (this->txFreq_ > 1000) {
        rslt |= 1;
        this->log("ERROR", "red", "M2006 %s: Max TxFreq is only 1000!",
                  this->name_);
    }

    return rslt;
}
