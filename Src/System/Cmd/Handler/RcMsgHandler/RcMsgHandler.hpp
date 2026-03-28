/*
 * @Author: IRON.NY 2537933690@qq.com
 * @Date: 2026-03-21 22:41:53
 * @LastEditors: IRON.NY 2537933690@qq.com
 * @LastEditTime: 2026-03-22 14:16:18
 * @FilePath: \PinyDart\Src\System\Cmd\Handler\RcMsgHandler\RcMsgHandler.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "Bsp.hpp"
#include "Handler.hpp"
#include <memory>
#include "RemoteControl.hpp"
#include "DT7.hpp"

#define IS_KEY_PRESS(CODE, KEY) (((CODE) & (KEY)) == (KEY))

enum class RCDevType_e : uint8_t {
    DT7 = 0,
    ET08A = 1,
    VT13 = 2,
};


template <RCDevType_e DEV_TYPE> class RcMsgHandler final : public Handler {
    typedef struct {
        struct {
            float rx;
            float ry;
            float lx;
            float ly;
            float wheel;
        };
        struct {
            DT7::Sw_e rSwitch;
            DT7::Sw_e lSwitch;             
        };
    } RcMsg_t;
public:
    RcMsgHandler(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    void init(MsgBus_s *_bus, EventGroupHandle_t _event) final;
    void handle() final;
    void notify(Msg *_msg, QueueHandle_t _queue) final;

private:

    RcMsg_t rcMsg_ = {};
    MsgBus_s *msgBus_;

    ChassisMsg_s cMsg_ = {}; // sample
    GimbalMsg_s gmsg_;
    ShooterMsg_s smsg_;

    std::unique_ptr<RemoteControl> rc_ = nullptr;

    void createRcDev(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    
    void scheduleDaemon();

    void masterHandle();

};
