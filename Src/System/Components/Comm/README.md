# Comm (Multi-core communication) v1.0.0



## 更新日志

1. 创建本README



## How to use

### 在AppManager中启用

将rxTask和txTask分别注册在主任务的开头和末尾

```cpp
{
    // inner AppManager::initApp()
#if APP_USE_COMM
    schedule([]() { CommManager::instance().rxTask(); });
#endif
    
    // ... other schedule action
    
#if APP_USE_COMM
    schedule([]() { CommManager::instance().txTask(); });
#endif
}
```



### 定义你的包类型

主要内容：包长、包ID、包结构、包压缩动作、包解压动作

```cpp
// PacketImpl.hpp
class ChassisCtrlPacketType{
    static constexpr uint8_t LEN = 8; // just write your length number
    static constexpr uint16_t ID = 0x21; // packet's uid
    struct ProtoData_s{
        // your ProtoData definition
    }
#pragma pack(push, 1) // this is a neccessery preprocessing cmd
    union Data_u{
        struct Data_s{
            // your PacketData definition
        } content;
        uint8_t bytes[LEN];
    } data;
#pragma pack(pop) // this is a neccessery preprocessing cmd
    static Data_u compress(const ProtoData_s &_protoData){
        // your compress implementation
    }
    static ProtoData_s decompress(const Data_u &_data){
        // your decompress implementation
    }
};
```



### 创建 发送/接收 的包

在发送机构（Master）中继承创建你的发送包

在接收机构（Slave）中继承创建你的接收包

```cpp
// Standard.hpp
// this is a sample, gimbal is a master

class ChassisCtrlTxPacket : public COMM::TxPacket<COMM::ChassisCtrlPacketType> {
    void send(uint8_t *_buf, uint16_t _len) final;
};
```

```cpp
// Standard.cpp
void ChassisCtrlTxPacket::send(uint8_t *_buf, uint16_t _len)
{
    // if you want to communicate by CAN1
    Can::instance().transmitData(&HCAN1, uid(), _buf, _len);
}
```

```cpp
// Omni.hpp
// this is a sample, chassis is a slave

class ChassisCtrlRxPacket : public COMM::RxPacket<COMM::ChassisCtrlPacketType, 1> {
public:
    ChassisCtrlRxPacket() { registerCallback(); }
    void registerCallback() final;
};
```

```cpp
// Omni.cpp
void ChassisCtrlRxPacket::registerCallback()
{
    // if you want to communicate by CAN1
    Can::instance().registerCallback(
            &HCAN1, uid(), [this](const uint8_t *_rxBuf) {
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(this->queue_, _rxBuf,
                                  &higherPriorityTaskWoken);
            });
}
```

最后别忘了在你的机构中实例化他们

```cpp
class MyMasterActuators{
    // ...
    ChassisCtrlTxPacket chassisCtrlTxPacket_;
    // ...
};
```

```cpp
class MySlaveActuators{
    // ...
    ChassisCtrlRxPacket chassisCtrlRxPacket_;
    // ...
};
```



### 更新你的数据

发送包基类中有更新数据的API

```cpp
// TxPacket.hpp
void loadFull(ProtoData *_data){
    memcpy(&data_, &_data, sizeof(ProtoData));
}

ProtoData &setData() { return data_; }
```

在发送端（Master），调用API更新数据

```cpp
// you can use it like this in gimbal
{
    // This is the area for updating functions
    // ...
    ChassisMsg_s cmsg{};
    if (xQueueReceive((((MsgBus_s *)_param)->chassisQueue), &cmsg, 0) ==
        pdTRUE) {
    };
    chassisCtrlTxPacket_.setData().vx = cmsg.vx;
    chassisCtrlTxPacket_.setData().vy = cmsg.vy;
    chassisCtrlTxPacket_.setData().gimbalYaw = endYawAng_;
    // ...
}
```

