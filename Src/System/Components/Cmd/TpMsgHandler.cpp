#include "TpMsgHandler.hpp"
#include "Rc.hpp" // 获取遥控器状态判断是否在示教模式
#include "Pump.hpp"
#include "sdkconfig.h"

void TpMsgHandler::init(MsgBus_s *_bus, EventGroupHandle_t _event)
{
    msgBus_ = _bus;
    this->event = _event;
    extern UART_HandleTypeDef EXTENSION_UART_HANDLE;
    tp_.init(&EXTENSION_UART_HANDLE, _event);
}

void TpMsgHandler::handle()
{
    tp_.convert();
    TpCtrl_t tpData = tp_.getData();
    RC::RcRawMsg_t rcData = RC::Rc::instance().getData();

    // 直接更新 teachModeActive_，无需中间变量 isTeachMode
    if (rcData.rc.switchLeft == RC_SW_UP) {
        if (!teachModeActive && rcData.rc.ch1 == 660) {
            teachModeActive = true;
        } //首次激活后，即使右摇杆回位，TP 仍保持有效。
    } else {
        teachModeActive = false;
    }

    armMsg_.source = ControlSource_e::TP;
    armMsg_.state = State_e::STOP;

    if (teachModeActive) {
        //示教模式激活：生成有效指令
        armMsg_.state = State_e::TEACH;
        // 填充示教器的关节角度（绝对位置）
        armMsg_.j1 = tpData.joint[0];
        armMsg_.j2 = tpData.joint[1];
        armMsg_.j3 = tpData.joint[2];
        armMsg_.j4 = tpData.joint[3];
        armMsg_.j5 = tpData.joint[4];
        armMsg_.j6 = tpData.joint[5];
        armMsg_.j7 = tpData.joint[6];

        if (tpData.push == 1) {
            PUMP::Controller::instance().apply(PUMP::ALL_ON); // 气泵开
        } else {
            PUMP::Controller::instance().apply(PUMP::ALL_OFF); // 气泵关
        }
    }

    notify(&armMsg_, msgBus_->armQueue);
}

void TpMsgHandler::notify(Msg *_msg, QueueHandle_t _queue)
{
    xQueueSend(_queue, _msg, 0); // 非阻塞发送
}
