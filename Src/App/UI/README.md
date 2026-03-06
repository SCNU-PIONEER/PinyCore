# UI

## introduce


|    文件    | ui 层级 |                   功能                   |
| :------: | :---: | :------------------------------------: |
|  Config  |  配置层  |  用户自定ui信息  |
|   App    |  上层   |  freertos 实时任务, 接收外部模块的消息并更新动态 ui 数据 |
|  Client  |  中间层  |            实现 ui 数据优先级排序算法             |
| Protocol |  协议层  |                 裁判系统协议                 |
| Builder  |  底层   |             负责在发送的时候构建数据包              |
|  Sender  |  底层   |              真正负责串口发送数据包               |


## init

### 与RM_UI_Designer兼容

为了保证代码的安全性，在Client的协议中使用了强类型枚举，但是在builder中没有使用，就是为了与RM_UI_Designer的传统C风格代码兼容

### 要修改的地方

Designer.hpp: 配置UI的数量

```cpp
/* need to add */
static constexpr uint8_t DYNAIMIC_NUM = 3;
static constexpr uint8_t STATIC_NUM = 1;
```

Designer.cpp: 配置UI数据

```cpp
    _info[0].config = {
        .priority = UI::Priority_e::HIGH,
        .name = "ne",
        .uiType = Type_e::CHAR,
        .operateType = OperateType_e::ADD,
        .layer = 0,
        .color = Color_e::RED_BLUE,
        .width = 2,
        .startX = 76,
        .startY = 840,
        //直线，矩形，正圆，圆弧需要使用
        .endX = 600,
        .endY = 600,
        //正圆半径
        .radius = 100,
        //圆弧起始角度，终止角度
        .startAngle = 20,
        .endAngle = 300,
        //浮点数：整型数均为 32 位，对于浮点数，实际显示的值为输入的值/1000
        .floatNum = 100.f,
        .decimal = 0,
        //整型数数据
        .intNum = 1000,
        //字体大小
        .size = 11,
        .text = "1234567890",
    };
```
## 使用
menuconfig中开启UI后

在对应的模块中发送队列,如在底盘update中
```cpp
        UI::Msg_s uiMsg;
        ChassisUIMsg_s uiMsgData{ .state = msg.state };
        uiMsg.type = UI::Event_e::CHASSIS;
        uiMsg.pdata = &uiMsgData;
        xQueueSend(UI::UIAPP::instance()->rxQueue, &uiMsg, 0);
```

机器人ID会自动在RefereeHanlder更新