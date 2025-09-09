#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "MsgImpl.hpp"
#include "./UIClient.hpp"

namespace UI {

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 用户配置区 ━━━━━━━━━━━━━━━━━━━━━━━━━━

// 发送间隔时间(ms) 默认40ms (25HZ), 裁判系统上限是30HZ
static constexpr uint8_t SEND_INTERVAL = 40;

// 队列接收事件
enum class Event_e : uint8_t { CHASSIS, GIMBAL, ARM, REFEREE };

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

struct Msg_s {
    Event_e type;
    void *pdata;
};


class App {
public:
    App(uint8_t _id);
    void init();
    void update(const Msg_s *_param);
    void task();

protected:
    void updateChassis(const ChassisTxMsg_s *_msg);
    void updateGimbal(const GimbalTxMsg_s *_msg);
    void updateArm(const ArmTxMsg_s *_msg);
    void updateReferee(const RefereeTxMsg_s *_msg);

private:
    Client client_;
    Info_s *dynamicInfo_;
    Info_s *constInfo_;
    QueueHandle_t rxQueue;
    uint8_t queueLen_ = 0;
    uint32_t updateCnt = 0;
};

} // namespace UI
