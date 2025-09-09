/**
 * @file Client.cpp
 * @brief 通过优先队列实现UI优先级调度
 *
 * This file contains code from Priority_UI
 * Copyright (c) 2025 IsaacZH
 * Used under MIT License - https://opensource.org/licenses/MIT
 */

#include <cstring>
#include <cstdio>
#include "FreeRTOS.h"
#include "task.h"
#include "./UISender.hpp"
#include "./UIProtocol.hpp"
#include "./UIClient.hpp"

using namespace UI;

Client::Client(UART_HandleTypeDef _huart, uint8_t _id) : sender_(_huart, _id) {}

Status_e Client::initList(Info_s *_dynamicInfo, uint8_t _dynamicNum,
                          Info_s *_info, uint8_t _num)
{
    // 初始化动态UI链表
    Status_e res1 = initPriorityLinkedList(&dynamicListHead_, _dynamicInfo,
                                           _dynamicNum);
    // 初始化不变UI链表
    Status_e res2 = initPriorityLinkedList(&constListHead_, _info, _num);
    // 初始化两个链表，将dynamic_ui_info和const_ui_info两个数组中不是CHAR类型的UI存进其中一个链表，其他类型的UI存进另一个链表
    Status_e res3 = initTypeLinkedLists(&graphicListHead_, &charListHead_,
                                        _dynamicInfo, _info, _dynamicNum, _num);

    if (res1 == Status_e::ERROR || res2 == Status_e::ERROR ||
        res3 == Status_e::ERROR) {
        return Status_e::ERROR;
    } else {
        return Status_e::OK;
    }
}

void Client::sendInit()
{
    static uint8_t isIniting = 1;
    static uint8_t initTimes = 0;
    if (isIniting == 1) {
        if (sendForce() == Status_e::OK) {
            initTimes++;
        }
        if (initTimes >= PER_INIT_UI_TIMES) //初始化完PER_INIT_UI_TIMES次
        {
            isIniting = 0; //初始化完毕
            initTimes = 0;
        }
    }
}

Status_e Client::ready(Info_s *_info)
{
    if (_info == nullptr) {
        return Status_e::ERROR;
    }

    if (_info->sentState ==
        SendState_e::SENT) //发过先更新时间，未先之前一直某发又更新就会一直发唔出去
    {
        _info->updateTick = xTaskGetTickCount();
    }

    _info->sentState = SendState_e::NOT_SENT;

    return Status_e::OK;
}

uint32_t Client::calculatePriority(Info_s *_msg)
{
    uint32_t priorityValue = 0;
    uint32_t currentTick = xTaskGetTickCount();
    uint32_t age = currentTick - _msg->updateTick; //消息的年龄

    // 根据消息的优先级计算优先级值
    switch (_msg->config.priority) {
    case Priority_e::HIGH:
        priorityValue = static_cast<uint32_t>(PriorityWeight_e::HIGH);
        break;
    case Priority_e::MID:
        priorityValue = static_cast<uint32_t>(PriorityWeight_e::MID);
        break;
    case Priority_e::LOW:
        priorityValue = static_cast<uint32_t>(PriorityWeight_e::LOW);
        break;
    }

    // 未发送的消息加上等待时间加权
    if (_msg->sentState == SendState_e::NOT_SENT) {
        priorityValue += age;
    }

    return priorityValue;
}

Node_s *Client::sortedMerge(Node_s *_node1, Node_s *_node2)
{
    if (_node1 == nullptr)
        return (_node2);
    else if (_node2 == nullptr)
        return (_node1);

    _node1->ui->priorityValue = calculatePriority(_node1->ui);
    _node2->ui->priorityValue = calculatePriority(_node2->ui);

    Node_s *res = nullptr;
    bool node1First = false;

    // 优先判断消息是否发送(未发送的消息优先级最高)
    if (_node1->ui->sentState == SendState_e::NOT_SENT &&
        _node2->ui->sentState == SendState_e::SENT) {
        node1First = true;
    } else if (_node1->ui->sentState == SendState_e::SENT &&
               _node2->ui->sentState == SendState_e::NOT_SENT) {
        node1First = false;
    }
    // 发送状态相同，判断优先级
    else {
        node1First = (_node1->ui->priorityValue >= _node2->ui->priorityValue);
    }

    if (node1First) {
        res = _node1;
        res->next = sortedMerge(_node1->next, _node2);
    } else {
        res = _node2;
        res->next = sortedMerge(_node1, _node2->next);
    }

    return res;
}

void Client::frontBackSplit(Node_s *_source, Node_s **_frontRef,
                            Node_s **_backRef)
{
    Node_s *fast;
    Node_s *slow;
    slow = _source;
    fast = _source->next;

    // 使用快慢指针法来找到链表的中点
    // 快指针每次移动两个节点，慢指针每次移动一个节点
    // 当快指针到达链表的末尾时，慢指针就在链表的中点
    while (fast != nullptr) {
        fast = fast->next;
        if (fast != nullptr) {
            slow = slow->next;
            fast = fast->next;
        }
    }

    // 将链表分成两半
    // 前半部分的头节点是source，后半部分的头节点是slow->next
    *_frontRef = _source;
    *_backRef = slow->next;
    slow->next = nullptr;
}

/**
 * @brief 使用分治算法来对链表进行排序
 *
 * @param headRef
 */
void Client::mergeSort(Node_s **_headRef)
{
    Node_s *head = *_headRef;
    Node_s *a;
    Node_s *b;

    // 如果链表为空，或者链表只有一个节点，那么链表已经是排序的，直接返回
    if ((head == nullptr) || (head->next == nullptr)) {
        return;
    }

    // 使用FrontBackSplit函数将链表分成两半
    frontBackSplit(head, &a, &b);

    // 对每一半递归地进行归并排序
    mergeSort(&a);
    mergeSort(&b);

    // 使用SortedMerge函数将两个已排序的部分合并成一个完整的排序链表
    *_headRef = sortedMerge(a, b);
}

/**
 * @brief
 * @param headRef 链表头指针的地址
 * @param ui_input 要存进去的UI信息
 * @param num ui_info数组成员个数
 */
Status_e Client::initPriorityLinkedList(Node_s **_headRef, Info_s *_input,
                                        uint8_t _num)
{
    if (_headRef == nullptr) {
        return Status_e::ERROR; // 没有声明链表头指针
    }

    Node_s *newNode = nullptr;
    Node_s *cursor = *_headRef;
    Info_s *ui = _input;
    // 创建一个零结构体
    Info_s zeroStruct = {};
    // 遍历ui_input数组的其余元素，并将每个元素添加到链表中
    for (uint8_t i = 0; i < _num; i++) {
        if (*ui == zeroStruct) {
            // 创建一个新的节点
            newNode = new Node_s;
            if (newNode == nullptr) {
                return Status_e::ERROR; // heap太小，申请不了内存
            }
            newNode->ui = ui;
            newNode->next = nullptr;

            if (*_headRef == nullptr) //如果链表为空，将新节点设置为链表的头节点
            {
                *_headRef = newNode;
            } else {
                // 将新节点添加到链表的末尾
                cursor->next = newNode;
            }
            cursor = newNode;
        }
        // 增加指针以访问下一个元素
        ui++;
    }
    return Status_e::OK;
}

Status_e Client::initTypeLinkedLists(Node_s **_graphicLink, Node_s **_charLink,
                                     Info_s *_dynamicUiInfo, Info_s *_uiInfo,
                                     const uint8_t _dynamicNum,
                                     const uint8_t _num)
{
    if (_graphicLink == nullptr || _charLink == nullptr) {
        return Status_e::ERROR; // 没有声明链表头指针
    }

    Node_s *graphicLinkCursor = nullptr;
    Node_s *charLinkCursor = nullptr;

    // 创建一个零结构体
    Info_s zeroStruct;
    memset(&zeroStruct, 0, sizeof(Info_s));

    // 遍历dynamic_ui_info数组，将不是CHAR类型的UI添加到第一个链表中，其他类型的UI添加到第二个链表中
    Info_s *dynamicPtr = _dynamicUiInfo;
    for (uint8_t i = 0; i < _dynamicNum; i++) {
        if (*dynamicPtr == zeroStruct) {
            dynamicPtr++;
            continue;
        }
        //分配内存存入当前的UI信息
        Node_s *newNode = new Node_s;
        if (newNode == nullptr) {
            return Status_e::ERROR; // heap太小，申请不了内存
        }
        newNode->ui = dynamicPtr;
        newNode->next = nullptr;
//给当前UI命名
#ifdef AUTO_UI_NAME_ENABLE
        char *name = dynamicPtr->config.name;
        sprintf(name, "%d", i);
#endif
        //判断当前UI是否为CHAR类型
        if (dynamicPtr->config.uiType != Type_e::CHAR &&
            dynamicPtr->config.operateType != OperateType_e::DELETE) {
            if (*_graphicLink ==
                nullptr) //如果链表为空，将新节点设置为链表的头节点
            {
                *_graphicLink = newNode;
            } else //如果链表不为空，将新节点添加到链表的末尾
            {
                graphicLinkCursor->next = newNode;
            }
            graphicLinkCursor = newNode; //光标指向新节点
        } else if (dynamicPtr->config.operateType !=
                   OperateType_e::DELETE) //如果当前UI为CHAR类型
        {
            if (*_charLink ==
                nullptr) //如果链表为空，将新节点设置为链表的头节点
            {
                *_charLink = newNode; //将新节点设置为链表的头节点
            } else                    //如果链表不为空，将新节点添加到链表的末尾
            {
                charLinkCursor->next = newNode;
            }
            charLinkCursor = newNode; //光标指向新节点
        }
        dynamicPtr++;
    }

    // 遍历const_ui_info数组，将不是CHAR类型的UI添加到第一个链表中，其他类型的UI添加到第二个链表中
    Info_s *constPtr = _uiInfo;
    for (uint8_t i = 0; i < _num; i++) {
        if (*constPtr == zeroStruct) {
            constPtr++;
            continue;
        }
        //分配内存存入当前的UI信息
        Node_s *newNode = new Node_s;
        if (newNode == nullptr) {
            return Status_e::ERROR;
        }
        newNode->ui = constPtr;
        newNode->next = nullptr;
//给当前UI命名
#ifdef AUTO_UI_NAME_ENABLE
        char *name = constPtr->config.name;
        sprintf(name, "%d", i + _dynamicNum + 1);
#endif
        //判断当前UI是否为CHAR类型
        if (constPtr->config.uiType != Type_e::CHAR &&
            constPtr->config.operateType != OperateType_e::DELETE) {
            if (*_graphicLink ==
                nullptr) //如果链表为空，将新节点设置为链表的头节点
            {
                *_graphicLink = newNode;
            } else //如果链表不为空，将新节点添加到链表的末尾
            {
                graphicLinkCursor->next = newNode;
            }
            graphicLinkCursor = newNode;
        } else if (constPtr->config.operateType !=
                   OperateType_e::DELETE) //如果当前UI为CHAR类型
        {
            if (*_charLink ==
                nullptr) //如果链表为空，将新节点设置为链表的头节点
            {
                *_charLink = newNode;
            } else //如果链表不为空，将新节点添加到链表的末尾
            {
                charLinkCursor->next = newNode;
            }
            charLinkCursor = newNode;
        }
        constPtr++;
    }
    return Status_e::OK;
}


Status_e Client::storeHighPriorityUI(Node_s *_dynamicListHead,
                                     Node_s *_listHead,
                                     Info_s *_graphicPriorityBuf,
                                     Info_s *_charPriorityBuf,
                                     uint8_t *_graphicBufNum,
                                     SendMode_e *_sendMode)
{
    *_graphicBufNum = 0; //图形UI缓存个数清零

    uint8_t graphicCnt = 0;
    uint8_t bufSize = 7;
    Node_s *dynamicListCursor = _dynamicListHead;
    static Node_s *constListCursor;
    if (constListCursor == nullptr) {
        constListCursor = _listHead;
    }


    if (dynamicListCursor == nullptr && constListCursor == nullptr) {
        return Status_e::ERROR; // 跑到这里证明没有初始化链表
    }

    //第一个节点为字符，发送字符 PRIORITY_LOW的字符会在第一的时候发送
    if (dynamicListCursor->ui->config.uiType == Type_e::CHAR) {
        if (dynamicListCursor != nullptr) {
            *_charPriorityBuf = *dynamicListCursor->ui;
            dynamicListCursor->ui->sentState = SendState_e::SENT;
            *_sendMode = SendMode_e::CHAR;
            return Status_e::OK;
        }
    }

    // 指向链表的头节点不为空 且 数组下标小于数组大小 且 当前节点的发送状态为未发送
    if (dynamicListCursor != nullptr) {
        while (graphicCnt < bufSize &&
               dynamicListCursor->ui->sentState == SendState_e::NOT_SENT) {
            if (dynamicListCursor->ui->config.uiType != Type_e::CHAR) {
                _graphicPriorityBuf[graphicCnt] = *(
                        dynamicListCursor->ui); //将当前节点的ui信息存储到数组中
                dynamicListCursor->ui->sentState = SendState_e::SENT;
                graphicCnt++;        //数组下标自增
                (*_graphicBufNum)++; //图形UI缓存个数自增
            } else if (dynamicListCursor->ui->config.priority ==
                       Priority_e::HIGH) //如果当前节点为字符且优先级为高
            {
                if (graphicCnt <= HIGH_CHAR_PRIORITY_LEVEL) {
                    //将之前存入的图形信息的发送状态变回未发送
                    dynamicListCursor = _dynamicListHead;
                    for (uint8_t i = 0;
                         i < graphicCnt && dynamicListCursor->next != nullptr;
                         i++) {
                        dynamicListCursor->ui->sentState =
                                SendState_e::NOT_SENT;
                        dynamicListCursor = dynamicListCursor->next;
                    }
                    //把要发送的字符信息存入buffer
                    *_charPriorityBuf = *dynamicListCursor->ui;
                    dynamicListCursor->ui->sentState = SendState_e::SENT;
                    *_sendMode = SendMode_e::CHAR;
                    return Status_e::OK; //发送字符
                }
            } else if (dynamicListCursor->ui->config.priority ==
                       Priority_e::MID) //如果当前节点为字符且优先级为中
            {
                if (graphicCnt <=
                    MID_CHAR_PRIORITY_LEVEL) //存入图形buffer的个数小于5就发送字符
                {
                    //将之前存入的图形信息的发送状态变回未发送
                    dynamicListCursor = _dynamicListHead;
                    for (uint8_t i = 0;
                         i < graphicCnt && dynamicListCursor->next != nullptr;
                         i++) {
                        dynamicListCursor->ui->sentState =
                                SendState_e::NOT_SENT;
                        dynamicListCursor = dynamicListCursor->next;
                    }
                    //把要发送的字符信息存入buffer
                    *_charPriorityBuf = *dynamicListCursor->ui;
                    dynamicListCursor->ui->sentState = SendState_e::SENT;
                    *_sendMode = SendMode_e::CHAR;
                    return Status_e::OK; //发送字符
                }
            }

            //如果下一个节点为空，退出循环
            if (dynamicListCursor->next == nullptr) {
                break;
            }
            dynamicListCursor = dynamicListCursor->next; //指向下一个节点
        }
    }
    // 剩下部分填入不变UI
    if (constListCursor != nullptr) {
        while (graphicCnt < bufSize) {
            if (constListCursor->ui->config.uiType != Type_e::CHAR) {
                _graphicPriorityBuf[graphicCnt] =
                        *(constListCursor->ui); //将当前节点的ui信息存储到数组中
                _graphicPriorityBuf[graphicCnt].config.operateType =
                        OperateType_e::ADD;
                graphicCnt++;        //数组下标自增
                (*_graphicBufNum)++; //图形UI缓存个数自增
            }
            //如果下一个节点为空，退出循环
            if (constListCursor->next == nullptr) {
                constListCursor = _listHead;
                break;
            }
            constListCursor = constListCursor->next; //指向下一个节点
        }
    }
    *_sendMode = SendMode_e::GRAPHIC;
    return Status_e::OK; //发送图形
}


CharGraphicData_s Client::processCharInfo2Buffer(Info_s _info,
                                                 uint8_t _addOperateEnable)
{
    OperateType_e operateTpye;
    if (_addOperateEnable == 0) {
        operateTpye = _info.config.operateType;
    } else {
        operateTpye = OperateType_e::ADD;
    }

    CharGraphicData_s res = {};
    res.option =
            builder_.create(_info.config.name, operateTpye, _info.config.layer)
                    .color(_info.config.color)
                    .width(_info.config.width)
                    .character(_info.config.size, strlen(_info.config.text),
                               _info.config.startX, _info.config.startY)
                    .build();

    memcpy(&res.string, &_info.config.text, sizeof(_info.config.text));

    return res;
}

SevenGraphicData_s Client::processGraphic2Buffer(Info_s *_info,
                                                 uint8_t _infoSize,
                                                 uint8_t _addOperateEnable)
{
    SevenGraphicData_s res;
    Info_s *ui = _info;
    for (uint8_t i = 0; i < 7; i++) {
        //判断是否会越界
        if (i >= _infoSize) {
            return res;
        }
        //配置操作类型
        OperateType_e operateTpye;
        if (_addOperateEnable == 0) {
            operateTpye = ui->config.operateType;
        } else {
            operateTpye = OperateType_e::ADD;
        }
        //判断UI类型配置信息
        switch (ui->config.uiType) {
        case Type_e::LINE:
            res.data[i] = builder_.create(ui->config.name, operateTpye,
                                          ui->config.layer)
                                  .color(ui->config.color)
                                  .width(ui->config.width)
                                  .line(ui->config.startX, ui->config.startY,
                                        ui->config.endX, ui->config.endY)
                                  .build();
            break;
        case Type_e::CIRCLE:
            res.data[i] = builder_.create(ui->config.name, operateTpye,
                                          ui->config.layer)
                                  .color(ui->config.color)
                                  .width(ui->config.width)
                                  .circle(ui->config.startX, ui->config.startY,
                                          ui->config.radius)
                                  .build();
            break;
        case Type_e::RECTANGEL:
            res.data[i] = builder_.create(ui->config.name, operateTpye,
                                          ui->config.layer)
                                  .color(ui->config.color)
                                  .width(ui->config.width)
                                  .rectangle(ui->config.startX,
                                             ui->config.startY, ui->config.endX,
                                             ui->config.endY)
                                  .build();
            break;
        case Type_e::ELLIPSE:
            res.data[i] = builder_.create(ui->config.name, operateTpye,
                                          ui->config.layer)
                                  .color(ui->config.color)
                                  .width(ui->config.width)
                                  .ellipse(ui->config.startX, ui->config.startY,
                                           ui->config.endX, ui->config.endY)
                                  .build();
            break;
        case Type_e::ARC:
            res.data[i] = builder_.create(ui->config.name, operateTpye,
                                          ui->config.layer)
                                  .color(ui->config.color)
                                  .width(ui->config.width)
                                  .arc(ui->config.startAngle,
                                       ui->config.endAngle, ui->config.startX,
                                       ui->config.startY, ui->config.endX,
                                       ui->config.endY)
                                  .build();
            break;
        case Type_e::FLOAT:
            res.data[i] =
                    builder_.create(ui->config.name, operateTpye,
                                    ui->config.layer)
                            .color(ui->config.color)
                            .width(ui->config.width)
                            .floatNum(ui->config.size, ui->config.decimal,
                                      ui->config.startX, ui->config.startY,
                                      (int32_t)(ui->config.floatNum * 1000))
                            .build();
            break;
        case Type_e::INT:
            res.data[i] = builder_.create(ui->config.name, operateTpye,
                                          ui->config.layer)
                                  .color(ui->config.color)
                                  .width(ui->config.width)
                                  .intNum(ui->config.startX, ui->config.startY,
                                          ui->config.size, ui->config.intNum)
                                  .build();
            break;
        default:
            break;
        }
        ui++;
    }
    return res;
}

/**
 * @brief 正常发送UI
 *
 */
Status_e Client::send()
{
    SevenGraphicData_s graphicTxBuf = {}; // 图像发送缓存
    CharGraphicData_s charTxBuffer = {};  // 字符发送缓存
    //对动态UI链表进行排序
    mergeSort(&dynamicListHead_);
    //将优先级最高的UI信息存储到数组中
    if (storeHighPriorityUI(dynamicListHead_, constListHead_,
                            graphicPriorityBuf_, &charPriorityBuf_,
                            &graphicBufNum_, &sendMode_) == Status_e::ERROR) {
        return Status_e::ERROR; // 没有初始化链表
    }
    switch (sendMode_) {
    case SendMode_e::CHAR:
        charTxBuffer = processCharInfo2Buffer(charPriorityBuf_, 0);
        sender_.sendChar(charTxBuffer);
        break;
    case SendMode_e::GRAPHIC:
        graphicTxBuf =
                processGraphic2Buffer(graphicPriorityBuf_, graphicBufNum_, 0);
        sender_.sendSevenGraphic(graphicTxBuf);
        break;
    default:
        break;
    }
    return Status_e::OK;
}

Status_e Client::sendForce()
{
    static bool isSendCharFinishFlag = false;
    static bool isSendGraphicFinishFlag = false;

    static Node_s *graphicListCursor = nullptr;
    static Node_s *charListCursor = nullptr;

    /*判断graphic_list和charListHead_有某东西*******************************/
    if (graphicListHead_ == nullptr) {
        isSendGraphicFinishFlag = true; //某嘢就当发完了啰
    }
    if (charListHead_ == nullptr) {
        isSendCharFinishFlag = true; //某嘢就当发完了啰
    }
    /*判断是第一次进来，初始化光标******************************************/
    if (graphicListCursor == nullptr && graphicListHead_ != nullptr) {
        graphicListCursor = graphicListHead_;
    }
    if (charListCursor == nullptr && charListHead_ != nullptr) {
        charListCursor = charListHead_;
    }
    /*初始化发送缓存******************************************/
    SevenGraphicData_s graphicTxBuf; // 图像发送缓存
    CharGraphicData_s charTxBuf;     // 字符发送缓存
    Info_s graphicInfoBuf[7];        // 7个待发送图形信息
    /*优先发送字符******************************************/
    if (charListCursor != nullptr && !isSendCharFinishFlag) {
        if (charListCursor->next == nullptr) //如果到了链表尾部，就发送最后一次
        {
            charTxBuf = processCharInfo2Buffer(*charListCursor->ui, 1);
            sender_.sendChar(charTxBuf);
            isSendCharFinishFlag = true;
            return Status_e::BUSY;
        }
        charTxBuf = processCharInfo2Buffer(*charListCursor->ui, 1);
        sender_.sendChar(charTxBuf);
        charListCursor = charListCursor->next;
        return Status_e::BUSY;
    }
    /*发送图形******************************************/
    if (graphicListCursor != nullptr && !isSendGraphicFinishFlag) {
        //从链表中取出7个图形信息
        for (uint8_t i = 0; i < 7; i++) {
            if (graphicListCursor->next ==
                nullptr) //如果到了链表尾部，就发送完了
            {
                graphicInfoBuf[i] = *graphicListCursor->ui;
                isSendGraphicFinishFlag = true;
                graphicBufNum_ = i + 1;
                break;
            }
            graphicInfoBuf[i] = *graphicListCursor->ui;
            graphicListCursor = graphicListCursor->next;
            graphicBufNum_ = i + 1;
        }
        graphicTxBuf = processGraphic2Buffer(graphicInfoBuf, graphicBufNum_, 1);
        sender_.sendSevenGraphic(graphicTxBuf);
        return Status_e::BUSY;
    }
    /*判断是否都发完了******************************************/
    if (isSendCharFinishFlag && isSendGraphicFinishFlag) //都发完了
    {
        //复位，等待下一次发送
        charListCursor = charListHead_;       //回到头节点
        graphicListCursor = graphicListHead_; //回到头节点
        isSendCharFinishFlag = false;
        isSendGraphicFinishFlag = false;
        return Status_e::OK; //返回1，证明发送完了
    }

    return Status_e::ERROR; //唔应该跑到尼到
}

bool Info_s::operator==(const Info_s &_other) const
{
    return (sentState == _other.sentState && updateTick == _other.updateTick &&
            priorityValue == _other.priorityValue &&

            // 比较 config 成员
            config.priority == _other.config.priority &&
            config.uiType == _other.config.uiType &&
            strncmp(config.name, _other.config.name, 3) == 0 &&
            config.operateType == _other.config.operateType &&
            config.layer == _other.config.layer &&
            config.color == _other.config.color &&
            config.width == _other.config.width &&
            config.startX == _other.config.startX &&
            config.startY == _other.config.startY &&
            config.endX == _other.config.endX &&
            config.endY == _other.config.endY &&

            // 特殊配置
            config.radius == _other.config.radius &&
            config.startAngle == _other.config.startAngle &&
            config.endAngle == _other.config.endAngle &&
            config.size == _other.config.size &&
            config.floatNum == _other.config.floatNum &&
            config.decimal == _other.config.decimal &&
            config.intNum == _other.config.intNum &&
            strncmp(config.text, _other.config.text, 30) == 0);
}
