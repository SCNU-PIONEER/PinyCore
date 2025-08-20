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

```cpp
ui = new UI::App(UI_UART, UI_ROBOT_ID);
```

ui robot_id 可以在 Kconfig 中修改默认值，也可以在`void App::updateReferee(const RefereeTxMsg_s *_msg)` 中更新

## 与RM_UI_Designer兼容

为了保证代码的安全性，在Client的协议中使用了强类型枚举，但是在builder中没有使用，就是为了与RM_UI_Designer的传统C风格代码兼容

### 要修改的地方

Designer.hpp: 配置ui info的数量

```cpp
/* need to add */
static constexpr uint8_t UIdynamicNum = 3;
static constexpr uint8_t UIconstNum = 1;
```

App.cpp: ui info 需要赋值，若是动态的信息，需要设置优先级

```cpp
dynamicInfo_[0] = newConfig(ui_g_dynamic_NewArc);
dynamicInfo_[0].config.priority = Priority_e::HIGH;

dynamicInfo_[1] = newConfig(ui_g_dynamic_time);
dynamicInfo_[0].config.priority = Priority_e::MID;

dynamicInfo_[2] = newConfig(ui_g_dynamic_chassis_state);
dynamicInfo_[0].config.priority = Priority_e::LOW;


constInfo_[0] = newConfig(ui_g_static_chassis);
```
