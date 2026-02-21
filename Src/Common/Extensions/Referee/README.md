# Referee v1.0.0


## How to use
menuconfig中开启referee即可接收数据

裁判系统的数据保存在RefereeProt_s refereeData_;

```cpp
struct RefereeProt_s {
    GameStatus_s gameStatus;
    GameResult_s gameResult;
    GameRobotHP_s gameRobotHP;
    EventData_s eventData;
    RefereeWarning_s refereeWarning;
    DartRemainingTime_s dartRemainingTime;
    GameRobotStatus_s gameRobotStatus;
    PowerHeatData_s powerHeatData;
    GameRobotPos_s gameRobotPos;
    Buff_s buff;
    RobotHurt_s robotHurt;
    ShootData_s shootData;
    BulletRemaining_s bulletRemaining;
    RfidStatus_s rfidStatus;
    VtData_s vtData;
};
```


需要的数据在refereeHandler.cpp中添加
```cpp
void RefereeHandler::handle()
{
    ...
    
    msg_.bulletSpeed = rx.getRefereeData().shootData.bulletSpeed;
    msg_.shooterHeatLimit = rx.getRefereeData().gameRobotStatus.shooterHeatLimit;
    msg_.chassisPowerLimit = rx.getRefereeData().gameRobotStatus.chassisPowerLimit;
    msg_.chassisPowerBuffer = rx.getRefereeData().powerHeatData.chassisPowerBuffer;
    msg_.currentHP = rx.getRefereeData().gameRobotStatus.currentHP;
    msg_.rxFreq = rx.getRxFreq();

    ...
}
```


裁判系统发送只需调用
inline std::unique_ptr<REFEREE::Transmitter> refereeTx 中的send函数即可


## Update RefereeProt
裁判系统的串口协议封装在RefereeProt.hpp中，只需要修改该文件即可

手动更新
1. 对比新旧协议，修改各命令结构体变量

2. 如有新增cmdId，在CmdId_e中添加
```cpp
enum class CmdId_e : uint16_t {
    GAME_STATE = 0x0001,
    GAME_RESULT = 0x0002,
    GAME_ROBOT_HP = 0x0003,
    EVENT_DATA = 0x0101,
    -----------------
    RADAR_TARGET_POSITIONX = 0x0305,
    CUSTOMER_CTRL_TO_CLIENT_DATA = 0x0306,
    AUTO_ROBOT_TO_CLIENT_MAP = 0x0307,
    ROBOT_TO_CLIENT_MAP = 0x0308,
};
```

3. 如新增加的cmd或现有的cmd需要保存数据，在RefereeProt_s中添加
```cpp
struct RefereeProt_s {
    GameStatus_s gameStatus;
    GameResult_s gameResult;
    GameRobotHP_s gameRobotHP;
    EventData_s eventData;
    RefereeWarning_s refereeWarning;
    DartRemainingTime_s dartRemainingTime;
    GameRobotStatus_s gameRobotStatus;
    PowerHeatData_s powerHeatData;
    GameRobotPos_s gameRobotPos;
    Buff_s buff;
    RobotHurt_s robotHurt;
    ShootData_s shootData;
    BulletRemaining_s bulletRemaining;
    RfidStatus_s rfidStatus;
    VtData_s vtData;
};
```

然后在INFO中添加,注意一一对应，否则会数据错误
```cpp
INFO_s constexpr INFO[INFO_NUM] = {
    { .cmdId = CmdId_e::GAME_STATE,
      .offsetByte = 0,
      .size = sizeof(GameStatus_s) },
    { .cmdId = CmdId_e::GAME_RESULT,
      .offsetByte = offsetof(RefereeProt_s, gameResult),
      .size = sizeof(GameResult_s) },
    { .cmdId = CmdId_e::GAME_ROBOT_HP,
      .offsetByte = offsetof(RefereeProt_s, gameRobotHP),
      .size = sizeof(GameRobotHP_s) },
    -------------------------
    { .cmdId = CmdId_e::RFID_STATUS,
      .offsetByte = offsetof(RefereeProt_s, rfidStatus),
      .size = sizeof(RfidStatus_s) },
    { .cmdId = CmdId_e::VT_DATA,
      .offsetByte = offsetof(RefereeProt_s, vtData),
      .size = sizeof(VtData_s) },
};
```

脚本更新
使用脚本自动更新，脚本在tools/script/updateReferee.py中，使用方法如下

1. 使用vscode的AI插件（如Cline），识别新协议中的结构体和其变量，将结构体保存到tools/script/struct.json中

2. 终端运行 python tools/script/updateReferee.py --json-file "tools/script/structs.json"

AI提示词：
识别pdf的所有typedef _packed struct的结构体，忽略命令码0x301子内容ID的结构体
将识别到的结构体保存到structs.json中

"struct_definition": "uint32_t rfidStatus"和
"new_struct_name": "RfidStatus",里的要求驼峰命名法
