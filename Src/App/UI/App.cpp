#include "./App.hpp"
#include "Client.hpp"
#include "task.h"
#include "MsgImpl.hpp"
#include "./Designer.hpp"
#include "StmLog.hpp"
#include "cmsis_os2.h"
#include "sdkconfig.h"
#include <cstdint>

using namespace UI;

extern UART_HandleTypeDef UI_UART;

App::App(uint8_t _id) : client_(UI_UART, _id) { init(); }

void App::init()
{
    // 初始化动态UI和不变UI信息
    ui_init_g_dynamic();
    ui_init_g_static();

    /* need to config */
    dynamicInfo_[0] = newConfig(ui_g_dynamic_NewArc);
    dynamicInfo_[0].config.priority = Priority_e::HIGH;

    dynamicInfo_[1] = newConfig(ui_g_dynamic_time);
    dynamicInfo_[1].config.priority = Priority_e::MID;

    dynamicInfo_[2] = newConfig(ui_g_dynamic_chassis_state);
    dynamicInfo_[2].config.priority = Priority_e::LOW;


    constInfo_[0] = newConfig(ui_g_static_chassis);

    // 初始化UI链表
    if (client_.initList(dynamicInfo_, UIdynamicNum, constInfo_, UIconstNum) ==
        Status_e::ERROR) {
        LOG::error("UI", "List init failed");
    } else {
        LOG::info("UI", "List init success");
    }
    rxQueue = xQueueCreate(10, sizeof(Msg_s));
    client_.sendInit();

    // xTaskCreate(App::task, "UiTask", 128, this, osPriorityRealtime1, nullptr);

    LOG::info("UI", "task init success");
}

void App::update(const Msg_s *_msg)
{
    // 更新UI信息
    switch (_msg->type) {
    case Event_e::CHASSIS:
        updateChassis((ChassisTxMsg_s *)_msg->pdata);
        break;
    case Event_e::GIMBAL:
        updateGimbal((GimbalTxMsg_s *)_msg->pdata);
        break;
    case Event_e::ARM:
        updateArm((ArmTxMsg_s *)_msg->pdata);
        break;
    case Event_e::REFEREE:
        updateReferee((RefereeTxMsg_s *)_msg->pdata);
        break;
    default:
        break;
    }
}

void App::updateChassis(const ChassisTxMsg_s *_msg)
{
    // client_.ready(Event_e::CHASSIS);
}

void App::updateGimbal(const GimbalTxMsg_s *_msg)
{
    // client_.ready(Event_e::GIMBAL);
}

void App::updateArm(const ArmTxMsg_s *_msg)
{
    // client_.ready(Event_e::ARM);
}

void App::updateReferee(const RefereeTxMsg_s *_msg)
{
    //TODO:
    // client_.updateID(_msg->robotID);
    // client_.ready(Event_e::REFEREE);
}

void App::task()
{
    if (xTaskGetTickCount() - updateCnt >= SEND_INTERVAL) {
        updateCnt = xTaskGetTickCount();

        Msg_s param = {};
        if (xQueueReceive(rxQueue, &param, 0) == pdTRUE) {
            update(&param);
            client_.send();
        }
    }
}
