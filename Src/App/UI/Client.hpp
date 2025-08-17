/**
 * @file UIClient.hpp
 * @brief 通过优先队列实现UI优先级调度
 *
 * This file contains code from Priority_UI
 * Copyright (c) 2025 IsaacZH
 * Used under MIT License - https://opensource.org/licenses/MIT
 */

#pragma once

#include "./Protocol.hpp"
#include "./Sender.hpp"
#include "./Builder.hpp"
#include <cstdint>

namespace UI {

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 用户配置区 ━━━━━━━━━━━━━━━━━━━━━━━━━━
/*功能---------------------------------------------------*/
#define AUTO_UI_NAME_ENABLE // 自动命名

/*参数---------------------------------------------------*/
// 高优先级字符抢占图形UI的等级
static constexpr uint8_t HIGH_CHAR_PRIORITY_LEVEL = 7;
// 中优先级字符抢占图形UI的等级
static constexpr uint8_t MID_CHAR_PRIORITY_LEVEL = 5;

// 每次初始化的UI次数
static constexpr uint8_t PER_INIT_UI_TIMES = 1;

/**
 * @brief UI优先级
 */
enum class PriorityWeight_e : uint16_t {
    HIGH = 1000, // 高优先级权重
    MID = 500,   // 中优先级权重
    LOW = 0,     // 低优先级权重
};
//   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

/*
 * 优先级排序算法：
 * 高中低三种优先级有不同的权重，将该UI对应的优先级权重加上该UI被阻塞的时间，得到该UI的优先级值
 * 优先比较UI发送状态，没发送过的UI优先级比发送过的UI高，发送状态相同时，再比较优先级值
 * 高优先级的UI优先发送，当高优先级的UI太多阻塞低优先级UI过久时，低优先级的UI也可以被发送出去
 * 而且当在更新UI时，遍历到发送过的UI时就证明所有要更新的UI都已经发送过了。
 * 每次更新UI前都用归并排序对动态UI链表进行排序，保证高优先级的在前面。
 * 使用归并排序的原因是归并排序的时间复杂度恒为O(nlogn)。
*/

/**
 * @brief UI优先级
 */
enum class Priority_e : uint8_t {
    LOW = 0, // 低优先级(动态UI中不需要高刷新率的)
    MID,     // 中优先级(动态UI中需要一定刷新率的)
    HIGH,    // 高优先级(动态UI中需要高刷新率的)
};

/**
 * @brief 发送模式枚举
 */
enum class SendMode_e : uint8_t {
    CHAR = 0,
    GRAPHIC,
};

/**
 * @brief UI发送状态枚举
 */
enum class SendState_e : uint8_t {
    SENT = 0, // 消息已发送
    NOT_SENT, // 消息更新后未发送
};

enum class Type_e : uint8_t {
    LINE = 0,  // 直线
    RECTANGEL, // 矩形
    CIRCLE,    // 圆
    ELLIPSE,   // 椭圆
    ARC,       // 圆弧
    FLOAT,     // 浮点数
    INT,       // 整数
    CHAR,      // 字符
};

/**
 * @brief 函数返回状态
 */
enum class Status_e : uint8_t {
    ERROR = 0,
    OK,
    BUSY,
};

/**
 * @brief UI配置信息结构体
 * 
 */
struct Config_s {
    /*******不变配置*********/
    Priority_e priority; // UI优先级(仅动态UI需要配置)
    char name[3];        // 图形名称
    Type_e uiType;       // UI内容类型
    /*******通用配置********/
    OperateType_e operateType; // 操作类型
    uint32_t layer;            // 图层数，0~9
    Color_e color;             // 颜色
    uint32_t width;            // 线条宽度
    uint32_t startX;           // 起点 x 坐标
    uint32_t startY;           // 起点 y 坐标
    uint32_t endX;             // 终点 x 坐标
    uint32_t endY;             // 终点 y 坐标
    /*******特殊配置********/
    uint32_t radius;     // 圆：半径
    uint32_t startAngle; // 圆弧：起始角度
    uint32_t endAngle;   // 圆弧：终止角度
    uint32_t size;       // 文字数字：字体大小
    float floatNum;      // 浮点数: 显示的数字
    uint32_t decimal;    // 浮点数：小数位有效个数
    int32_t intNum;      // 整数: 显示的数字
    char text[30];       // 字符串：显示的文字
};

/**
 * @brief UI信息结构体
 */
struct Info_s {
    SendState_e sentState;  // 消息发送状态标志位
    uint32_t updateTick;    // 更新消息时的时间戳
    uint16_t priorityValue; // UI的优先级值
    Config_s config;        // 用户配置UI信息

    bool operator==(const Info_s &_other) const;
};

/**
 * @brief UI链表节点结构体
 */
struct Node_s {
    Info_s *ui;
    struct Node_s *next;
};

class Client {
public:
    Client(UART_HandleTypeDef _huart, uint8_t _id);
    /**
    * @brief 初始化UI链表 在外部调用 一定要在Ui_Send之前调用
    */
    Status_e initList(Info_s *_dynamicInfo, uint8_t _dynamicNum, Info_s *_info,
                      uint8_t _num);

    /**
    * @brief 发送初始化 UI
    */
    void sendInit();

    /**
    * @brief 发送UI 在外部调用
    *
    */
    Status_e send();

    /**
    * @brief 更新完UI信息后调用此函数将UI设为准备发送状态
    */
    Status_e ready(Info_s *_info);

    /**
    * @brief 更新机器人ID
    */
    void updateID(uint16_t _id) { sender_.infoUpdate(_id); }

protected:
    /**
    * @brief 计算消息的优先级
    * @param msg
    * @note 未更新低优先级阻塞1000ms等于未被阻塞高优先级
    * @return priority_value
    */
    uint32_t calculatePriority(Info_s *_msg);

    /*合并两个有序链表*/
    Node_s *sortedMerge(Node_s *_node1, Node_s *_node2);

    /**
    * @brief 用于将一个链表分成两半,在mergeSort中调用
    * @param source 表头地址
    * @param frontRef 用于储存前半段的开始地址
    * @param backRef 用于储存后半段的开始地址
    */
    void frontBackSplit(Node_s *_source, Node_s **_frontRef, Node_s **_backRef);

    /*使用分治算法来对链表进行排序*/
    void mergeSort(Node_s **_headRef);

    /**
    * @brief 将链表中优先级高的ui结构体存储到一个数组中，
    *        在调用此函数前应该先调用mergeSort函数对链表进行排序
    *
    * @param dynamic_list_head 动态UI链表的头节点
    * @param const_list_head 不变UI链表的头节点
    * @param graphic_buffer 存储ui结构体的数组
    * @param character_buffer 存储字符ui结构体的数组
    * @param ui_graphic_buffer_num 图形UI缓存个数
    * @param ui_send_mode 发送模式
    * @return Status_e UI_ERROR：链表为空,没有初始化链表
    */
    Status_e storeHighPriorityUI(Node_s *_dynamicListHead, Node_s *_listHead,
                                 Info_s *_graphicPriorityBuf,
                                 Info_s *_charPriorityBuf,
                                 uint8_t *_graphicBufNum,
                                 SendMode_e *_sendMode);


    /*初始化优先队列*/
    Status_e initPriorityLinkedList(Node_s **_headRef, Info_s *_ui_input,
                                    uint8_t _num);

    /**
    * @brief 初始化两个链表，将dynamic_ui_info和const_ui_info两个数组中
    * 不是CHAR类型的UI存进其中一个链表，其他类型的UI存进另一个链表
    *
    * @param graphic_link 存放图像链表头指针的地址
    * @param char_link 存放字符链表头指针的地址
    * @param dynamic_ui_info 动态UI信息数组
    * @param const_ui_info 静态UI信息数组
    * @param dynamic_num 动态UI信息数组成员个数
    * @param const_num 静态UI信息数组成员个数
    */
    Status_e initTypeLinkedLists(Node_s **_graphicLink, Node_s **_charLink,
                                 Info_s *_dynamicUiInfo, Info_s *_uiInfo,
                                 const uint8_t _dynamicNum, const uint8_t _num);

    /**
    * @brief 配置字符信息进发送结构体
    *
    * @param ui_info UI信息结构体
    * @param add_operate_enable 1：强行ADD 0：按照UI配置的operate_type
    * @return ext_client_custom_character_t 配置好的发送结构体
    */
    CharGraphicData_s processCharInfo2Buffer(Info_s _info,
                                             uint8_t _addOperateEnable);

    /**
    * @brief 配置图形信息进发送结构体
    *
    * @param ui_info UI信息结构体
    * @param add_operate_enable 1：强行ADD 0：按照UI配置的operate_type
    * @return ext_client_custom_graphic_seven_t 配置好的发送结构体
    */
    SevenGraphicData_s processGraphic2Buffer(Info_s *_info, uint8_t _infoSize,
                                             uint8_t _addOperateEnable);

    /**
    * @brief UI强行发送ADD
    * @note  由于选手端登陆后，每个UI都需要发一次ADD，所以需要强制发送一次ADD
    */
    Status_e sendForce();

private:
    Node_s *dynamicListHead_; // 动态UI链表头
    Node_s *constListHead_;   // 不变UI链表头
    Node_s *graphicListHead_; // 图形UI链表头
    Node_s *charListHead_;    // 字符UI链表头

    Info_s graphicPriorityBuf_[7]; // 优先级最高的7个图形
    Info_s charPriorityBuf_;       // 优先级最高的字符

    SendMode_e sendMode_;       // 发送模式
    uint8_t graphicBufNum_ = 0; // 图形UI缓存个数

    Sender sender_;
    GraphicBuilder builder_;
};

} // namespace UI
