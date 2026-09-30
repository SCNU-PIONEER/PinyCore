# Message Center

# 功能简述

- `MessageCenter` 汇集应用层的数据，并为业务模块提供只读数据访问接口，在业务模块中使用消息中心单例获取数据；
- 消息中心获取遥控器，裁判系统，传感器，双板通信等数据，并进行数据预处理，将处理后的数据提供给业务模块使用；业务模块数据更新不再兼顾以上几类的数据处理，只负责更新状态机与电机数据，更易于维护。

# 目录结构

```text
MessageCenter/
├── Kconfig
├── MessageCenter.hpp       # 统一访问入口与对外数据接口
└── Board/
    ├── MasterMC/
    │   ├── MasterMC.hpp    # 上板数据收发包定义
    │   └── MasterMC.cpp    # 上板数据周期更新
    └── SlaveMC/
        ├── SlaveMC.hpp     # 下板数据收发包和外设适配
        └── SlaveMC.cpp     # 下板数据周期更新
```



# 说明

## 数据包的定义

### MasterMC上板消息中心

`MasterMC.hpp` 中定义的上板数据收发包主要包括：
- `Comm`发送给底盘与拨盘的控制数据包，视觉数据收发包；
```cpp
class TxChassisCmdPacket : public COMM::TxPacket<COMM::ChassisCtrlPacket> {
public:
    TxChassisCmdPacket(float _txfreq) : COMM::TxPacket<COMM::ChassisCtrlPacket>(_txfreq) {}
    void transmit(uint8_t *_buf, uint16_t _len) final { Can::instance().transmitBrsData(&HCAN3, uid(), _buf, _len); };
};
```
- `Topic` 订阅imu或模块状态；
- `Cmd` 遥控器数据包；
```cpp
class MasterMC {
public:
    MasterMC() {}
    ~MasterMC() = default;
    void masterUpdate(void *_param);

private:
    friend class MessageCenter;

    //cmd
    GimbalMsg_s gimbalMsg_ = {};
    //topic
    INS_SYS::INSData_s insMsg_ = {};
    Subscriber<INS_SYS::INSData_s> *insSub;
};
```
- 在构造函数列表初始化订阅者与发布者对象，函数体内注册双板通信接收数据包的中断回调；

### SlaveMC下板消息中心

`SlaveMC.hpp` 中定义的下板数据收发包主要包括：
- `Comm` 接收底盘与拨盘的控制数据包，视觉数据收发包；
- `Cmd` 裁判系统数据包；
- 在构造函数列表初始化订阅者与发布者对象，函数体内注册双板通信接收数据包的中断回调；

## 数据处理
在对应消息中心的update函数中自定义数据处理逻辑。

```cpp
void MasterMC::masterUpdate(void *_param)
{
    //cmd
    if (xQueueReceive(((MsgBus_s *)_param)->chassisQueue, &chassisMsg, 0) == pdTRUE) {
    }
    //topic
    insSub->receive();
    //comm
    COMM::DialCtrlPacket::ProtoData_s dialData = {};
    dialData.state = dialMsg_.state;
    dialData.isFire = dialMsg_.isFire && (gimbalMsg_.useVision ? vision.data().fire : true);
    dialData.isGimbalLaunched= gimbalStateMsg.isLaunched;
    dialTxPacket.setData() = dialData;
}
```
## 只读数据访问接口

需要在`MessageCenter.hpp` 提供数据访问接口。

```cpp
const GimbalMsg_s &gimbalMsg() { return masterMC.gimbalMsg_; };
```

## 业务模块中调用

使用消息中心的单例调用数据访问接口，获取数据。

例如：在底盘的更新函数中获取模块数据。
```cpp
 void update()
    {   

        msg.state=MessageCenter::instance().chassisData().state;
        msg.vx=MessageCenter::instance().chassisData().vx;
        msg.vy=MessageCenter::instance().chassisData().vy;
        deltaYaw_=MessageCenter::instance().chassisData().deltaYaw;
  
        updateSelf();
        this->stateFactory_.update();
    }
```



