# BoardComm v1.0.0

## 类与变量
1. class BoardManager：板间通信管理器，负责创建Board对象，并管理Board对象

   Board *board; //当前Board对象 

2. class Board：板间通信对象，负责消息的发送与接收

   MsgBus_u msgBus; //消息总线，用于存储BoardMsg指针

   MsgConfig_s msgConfig; //消息配置，用于存储需要发送的消息的配置

   BoardType_e boardType; //当前Board的类型，Master或Slave

   BoardId_e id; //当前Board的ID(MASTER_ID或SLAVE_ID)

   ChassisMsg_s chassisMsg_;
   Subscriber<ChassisMsg_s> *chassisSub_;
   Publisher<ChassisMsg_s> *chassisPub_;
   用于需要发送消息的Board订阅消息，需要接收消息的Board发布消息
   以上三个根据自己的需要发送的消息配置

3. class BoardMsg：消息对象
   uint8_t offsetId; //消息的偏移ID，用于与消息配置中的ID配对
   QueueHandle_t rxQueue_; //接收消息的队列
   RxBus_s::CANRxBuf_s<8> rxBuf_ = {}; //接收消息的缓冲区

## 板间通信逻辑
1. kconfig中配置Master or Slave，BoardManager根据配置将Board创建为Master或Slave对象，将Board的ID设置
2. Board中有消息总线MsgBus_u msgBus和消息配置MsgConfig_s msgConfig
   消息总线中拥有多个BoardMsg指针，每个BoardMsg初始化时有着自身的offsetId
   消息配置是需要发送的消息的配置，其中包含消息ID，消息内容，消息发送频率

发送：
任务中调用sendMessage()后循环遍历msgBus，调用BoardMsg的send函数，
根据配置的offsetId与消息配置结构体中的msgConfig的.base.id配对，将需要发送的消息传入发送

接收：
1. 构造函数中调用registerMsg，根据boardType决定传入的id，注册can回调
2. can回调中根据id找到对应的boardMsg，进入praseMessage函数，
3. 根据boardMsg的offsetId和传入的boardType与消息配置结构体中的msgConfig的.base.id配对，
   memcpy到对应msgConfig的.base.msg中

## how to use
1. 配置消息(Message.hpp)
    struct __attribute__((packed)) First_s {
        uint8_t vx : 4;
        uint8_t vy : 4;
        uint8_t yaw : 4;
        uint8_t robot_control_flag : 1;
        uint64_t reserved : 51;
    };
    然后将其填入MsgConfig_s中
    struct {
        MsgBase_s base;
        MASTER_MSG::First_s msg;
    } masterFirMsg;

    如需第三个包，则新创建struct __attribute__((packed)) Third_s 

    然后填入MsgConfig_s中

2. 配置消息MsgBase_s(BoardComm.hpp)
    MsgConfig_s msgConfig = {
        .masterFirMsg{ .base{
                .id = MASTER_ID + 0x01,
                .txFreq = 1000,
        } },
        .slaveFirMsg{ .base{
                .id = SLAVE_ID + 0x01,
                .txFreq = 1000,
        } },
    };
    .base.id为消息ID，.base.txFreq为发送频率

    新增的包需要在这里配置

3. 检查MsgBus_u(BoardComm.hpp)
   union MsgBus_u {
        struct {
            BoardMsg *first;
            BoardMsg *second;
            BoardMsg *third;
        };
        BoardMsg *_[3];
    };

    此时说明一个板可以发送三个包，如果有四个包需要新增

4. 装填消息（BoardComm.cpp）
    在Board::sendMessage函数中
   
    if constexpr (BOARD_MASTER) {
        if (chassisSub_->receive()) {
            //msgConfig.masterFirMsg.msg.vx = chassisMsg_.vx;
        }
    }
    else if constexpr (BOARD_SLAVE) {
    }

    如有需要的其他消息，需要配置Publisher和Subscriber，然后在这里调用receive()

5. 接收消息（BoardComm.cpp）
    在Board::praseMessage函数中

    if constexpr (BOARD_MASTER) {
        if (msgId == _msgConfig->slaveFirMsg.base.id) {
            memcpy(&_msgConfig->slaveFirMsg.msg, _rxBuf, 8);
        }
    }
    else if constexpr (BOARD_SLAVE) {
    }

    注意Master判断的是Slave的包id，Slave判断的是Master的包id
    如有新增的包需要在这里配置

6. 发布消息（BoardComm.cpp）
   
    void Board::msgPublish()
    {
        //chassisMsg_.vx = msgConfig.masterFirMsg.msg.vx;
        chassisPub_->publish();
    }
    对应配置

## 注意
1. static constexpr uint16_t MASTER_ID = 0x20;
   static constexpr uint16_t SLAVE_ID = 0x30;

   MASTER_ID和SLAVE_ID可以根据实际需求修改

2. 目前只兼容双板，但是可以扩展
    - enum class BoradType_e : uint8_t {
        MASTER,
        SLAVE,
    };  在这新增Tpye定义
    - 添加对应ID（如MASTER_ID）
    - 增加if constexpr (BOARD_MASTER)的部分
    - 增加Board构造函数中switch (boardType)的case
    - Board::registerBoard()i->registerMsg(pComHandle, (boardType == BoradType_e::MASTER) ?
                                           SLAVE_ID :
                                           MASTER_ID);
      增加传入id选择
```

