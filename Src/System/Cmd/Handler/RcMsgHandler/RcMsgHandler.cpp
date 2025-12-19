#include "RcMsgHandler.hpp"
#include "sdkconfig.h"

#include "remoteControl.hpp"

#ifdef CHASSIS_TYPE
#include CHASSIS_FILE
#endif
#ifdef GIMBAL_TYPE
#include GIMBAL_FILE
#endif

#if EXTENSION_DT7 == 1
#include "DT7.hpp"
#elif EXTENSION_ET08A == 1
#include "ET08A.hpp"
#elif EXTENSION_VT13 == 1
#include "VT13.hpp"
#endif

#include <cmath>
#include <cstring>
#include <bitset>


RcMsgHandler::RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event)
{
#if EXTENSION_DT7 == 1
    rc = std::make_unique<DT7>(_huart, _event, this->bit_);
#elif EXTENSION_ET08A == 1
    rc = std::make_unique<ET08A>(_huart, _event, this->bit_);
#elif EXTENSION_VT13 == 1
    rc = std::make_unique<VT13>(_huart, _event, this->bit_);
#endif
};

void RcMsgHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
}

void RcMsgHandler::handle()
{
    rc->parse();

    masterHandle();
}

/**
 * @brief Master handler for processing remote controller messages
 * 
 * @details Since remote controllers vary widely in channel configurations and button mappings
 *          a universal processing method cannot be implemented. Users should acquire data from 
 *          their specific remote controller, normalize it as needed, and implement custom
 *          interaction logic based on the application requirements.
 */
void RcMsgHandler::masterHandle() {}

void RcMsgHandler::notify(Msg *_msg, QueueHandle_t _queue) { xQueueSend(_queue, _msg, 0); }
