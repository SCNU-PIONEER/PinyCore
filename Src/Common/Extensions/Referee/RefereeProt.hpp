#pragma once

#include <cstddef>
#include <cstdint>

/* RFID功能卡 */
#define CARD_ATTACK     ((uint8_t)0x00)
#define CARD_PROTECT    ((uint8_t)0x01)
#define CARD_BLOOD_RED  ((uint8_t)0x02)
#define CARD_BLOOD_BLUE ((uint8_t)0x03)
#define CARD_HEAL_RED   ((uint8_t)0x04)
#define CARD_HEAL_BLUE  ((uint8_t)0x05)
#define CARD_COLD_RED   ((uint8_t)0x06)
#define CARD_COLD_BLUE  ((uint8_t)0x07)
#define CARD_FORT       ((uint8_t)0x08)

namespace REFEREE {

#pragma pack(push, 1)

struct FrameHeader_s {
    uint8_t sof;
    uint16_t dataLen;
    uint8_t seq;
    uint8_t crc8;
};

/* 比赛状态数据：0x0001。发送频率：1Hz */
struct GameStatus_s {
    uint8_t gameType : 4;
    uint8_t gameProgress : 4;
    uint16_t stageRemainTime;
    uint64_t syncTimeStamp;
};

/* 比赛结果数据：0x0002。发送频率：比赛结束后发送 */
struct GameResult_s {
    uint8_t winner;
};

/* 机器人血量数据：0x0003。发送频率：3Hz */
struct GameRobotHP_s {
    uint16_t red1RobotHP;
    uint16_t red2RobotHP;
    uint16_t red3RobotHP;
    uint16_t red4RobotHP;
    uint16_t reserved1;
    uint16_t red7RobotHP;
    uint16_t redOutpostHP;
    uint16_t redBaseHP;
    uint16_t blue1RobotHP;
    uint16_t blue2RobotHP;
    uint16_t blue3RobotHP;
    uint16_t blue4RobotHP;
    uint16_t reserved2;
    uint16_t blue7RobotHP;
    uint16_t blueOutpostHP;
    uint16_t blueBaseHP;
};

/* 场地事件数据：0x0101。发送频率：1Hz */
struct EventData_s {
    uint32_t eventData;
};

/* 裁判警告信息：0x0104。发送频率：己方警告发生后发送 */
struct RefereeWarning_s {
    uint8_t level;
    uint8_t offendingRobotId;
    uint8_t count;
};

/* 飞镖发射口倒计时：0x0105。发送频率：1Hz */
struct DartRemainingTime_s {
    uint8_t dartRemainingTime;
    uint16_t dartInfo;
};

/* 比赛机器人状态：0x0201。发送频率：10Hz */
struct GameRobotStatus_s {
    uint8_t robotId;
    uint8_t robotLevel;
    uint16_t currentHP;
    uint16_t maximumHP;
    uint16_t shooterCoolingValue;
    uint16_t shooterHeatLimit;
    uint16_t chassisPowerLimit;
    uint8_t GimbalOutput : 1;
    uint8_t ChassisOutput : 1;
    uint8_t ShooterOutput : 1;
};

/* 实时功率热量数据：0x0202。发送频率：10Hz */
struct PowerHeatData_s {
    uint16_t reserved1;
    uint16_t reserved2;
    float reserved3;
    uint16_t chassisPowerBuffer;
    uint16_t shooterId1_17mmCoolingHeat;
    uint16_t shooterId2_17mmCoolingHeat;
    uint16_t shooterId1_42mmCoolingHeat;
};

/* 机器人位置：0x0203。发送频率：1Hz */
struct GameRobotPos_s {
    float x;
    float y;
    float angle;
};

/* 机器人增益：0x0204。发送频率：3Hz */
struct Buff_s {
    uint8_t recoveryBuff;
    uint8_t coolingBuff;
    uint8_t defenceBuff;
    uint8_t vulnerabilityBuff;
    uint16_t attackBuff;
    uint8_t remainingEnergy;
};
/* 0x0206 伤害状态数据，伤害发生后发送*/
struct RobotHurt_s {
    uint8_t armorId : 4;
    uint8_t reason : 4;
};

/* 0x0207 实时射击数据，子弹发射后发送*/
struct ShootData_s {
    uint8_t bulletType;
    uint8_t shooterId;
    uint8_t bulletFreq;
    float bulletSpeed;
};

/* 0x0208 允许发弹量，固定以 10Hz 频率发送对应机器人*/
struct BulletRemaining_s {
    uint16_t bulletRemainingNum17mm;
    uint16_t bulletRemainingNum42mm;
    uint16_t coinRemainingNum;
    uint16_t bulletRemainingNumFortress; //新增，用于反馈堡垒提供剩余子弹数量
};

/* 0x0209 机器人 RFID 状态，3Hz 周期发送 该RFID持有机器人*/
struct RfidStatus_s {
    uint32_t rfidStatus;
};

/* 0x020A 飞镖发射器状态，3Hz 周期发送 */
struct DartClientCmd_s {
    uint8_t dartLaunchOpeningStatus;
    uint8_t reserved1;
    uint16_t targetChangeTime;
    uint16_t latestLaunchCmdTime;
};

/* 0x020B 己方地面机器人位置，1Hz 周期发送 */
struct GroundRobotPosition_s {
    float heroX;
    float heroY;
    float engineerX;
    float engineerY;
    float standard3X;
    float standard3Y;
    float standard4X;
    float standard4Y;
    float reserved1;
    float reserved2;
};

/* 0x020C 雷达易伤标记数据，1Hz 周期发送 */
struct RadarMarkData_s {
    uint8_t markProgress;
};

/* 0x020D 哨兵兑远程换数据，1Hz 周期发送 */
struct SentryInfo_s {
    uint32_t sentryInfo;
    uint16_t sentryInfo2;
};

/* 0x020E 雷达易伤次数，1Hz 周期发送 */
struct RadarInfo_s {
    uint8_t radarInfo;
};

/* 0x0301 机器人交互数据，30Hz 周期发送 */
struct RobotInteractionData_s {
    uint16_t dataCmdId;
    uint16_t senderId;
    uint16_t receiverId;
    uint8_t userData[112];
};

/* 0x0303 选手下发数据，1Hz 周期发送 */
struct MapCommand_s {
    float targetPositionX;
    float targetPositionY;
    uint8_t cmdKeyboard;
    uint8_t targetRobotId;
    uint16_t cmdSource;
};

/* 0x0305 小地图数据，1Hz 周期发送 */
struct MapRobotData_s {
    uint16_t heroPositionX;
    uint16_t heroPositionY;
    uint16_t engineerPositionX;
    uint16_t engineerPositionY;
    uint16_t infantry_3PositionX;
    uint16_t infantry_3PositionY;
    uint16_t infantry_4PositionX;
    uint16_t infantry_4PositionY;
    uint16_t infantry_5PositionX;
    uint16_t infantry_5PositionY;
    uint16_t sentryPositionX;
    uint16_t sentryPositionY;
};

/* 0x0306 自定义模拟键鼠，30Hz 周期上限 */
struct CustomClientData_s {
    uint16_t keyValue;
    uint16_t xPosition : 12;
    uint16_t mouseLeft : 4;
    uint16_t yPosition : 12;
    uint16_t mouseRight : 4;
    uint16_t reserved;
};

/* 0x0307 半自动数据，1Hz 周期发送 */
struct MapData_s {
    uint8_t intention;
    uint16_t startPositionX;
    uint16_t startPositionY;
    int8_t deltaX[49];
    int8_t deltaY[49];
    uint16_t senderId;
};

/* 0x0302 自定义控制器数据（向机器人发送），1Hz 周期发送 */
struct CustomRobotData_s {
    uint8_t data[30];
};

/* 0x0304 键鼠数据（图传链路），30Hz 周期发送 */
struct VtData_s {
    int16_t mouseX;
    int16_t mouseY;
    int16_t mouseZ;
    int8_t leftButtonDown;
    int8_t rightButtonDown;
    uint16_t keyboardValue;
    uint16_t reserved;
};

/* 0x0308 机器人向任意选手端发送，3Hz 周期接收 */
struct CustomInfo_s {
    uint16_t senderId;
    uint16_t receiverId;
    uint8_t userData[30];
};


/* 0x0309 自定义控制器数据（机器人发送），10Hz接收 */
struct RobotCustomData_s {
    uint8_t data[30];
};

uint8_t constexpr INFO_NUM = 20;

enum class CmdId_e : uint16_t {
    GAME_STATE = 0x0001,
    GAME_RESULT = 0x0002,
    GAME_ROBOT_HP = 0x0003,
    EVENT_DATA = 0x0101,
    REFEREE_ALERT = 0x0104,
    DART_COUNTDOWN = 0x0105,
    GAME_ROBOT_STATE = 0x0201,
    POWER_HEAT_DATA = 0x0202,
    GAME_ROBOT_POS = 0x0203,
    BUFF_MUSK = 0x0204,
    ROBOT_HURT = 0x0206,
    SHOOT_DATA = 0x0207,
    BULLET_REMAINING_NUM = 0x0208,
    RFID_STATUS = 0x0209,
    DART_CLIENT_DATA = 0x020A,
    GAME_GROUND_ROBOT_POS = 0x020B,
    RADAR_MARKING_DATA = 0x020C,
    SENTRY_AUTONOMOUS_SYN = 0x020D,
    RADAR_AUTONOMOUS_SYN = 0x020E,
    ROBOT_INTERACTION_DATA = 0x0301,
    CUSTOM_ROBOT_DATA = 0x0302,
    MAP_COMMAND = 0x0303,
    VT_DATA = 0x0304,
    RADAR_TARGET_POSITIONX = 0x0305,
    CUSTOMER_CTRL_TO_CLIENT_DATA = 0x0306,
    AUTO_ROBOT_TO_CLIENT_MAP = 0x0307,
    ROBOT_TO_CLIENT_MAP = 0x0308,
};

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
struct INFO_s {
    CmdId_e cmdId;
    size_t offsetByte;
    size_t size;
};

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
    { .cmdId = CmdId_e::EVENT_DATA,
      .offsetByte = offsetof(RefereeProt_s, eventData),
      .size = sizeof(EventData_s) },
    { .cmdId = CmdId_e::REFEREE_ALERT,
      .offsetByte = offsetof(RefereeProt_s, refereeWarning),
      .size = sizeof(RefereeWarning_s) },
    { .cmdId = CmdId_e::DART_COUNTDOWN,
      .offsetByte = offsetof(RefereeProt_s, dartRemainingTime),
      .size = sizeof(DartRemainingTime_s) },
    { .cmdId = CmdId_e::GAME_ROBOT_STATE,
      .offsetByte = offsetof(RefereeProt_s, gameRobotStatus),
      .size = sizeof(GameRobotStatus_s) },
    { .cmdId = CmdId_e::POWER_HEAT_DATA,
      .offsetByte = offsetof(RefereeProt_s, powerHeatData),
      .size = sizeof(PowerHeatData_s) },
    { .cmdId = CmdId_e::GAME_ROBOT_POS,
      .offsetByte = offsetof(RefereeProt_s, gameRobotPos),
      .size = sizeof(GameRobotPos_s) },
    { .cmdId = CmdId_e::BUFF_MUSK,
      .offsetByte = offsetof(RefereeProt_s, buff),
      .size = sizeof(Buff_s) },
    { .cmdId = CmdId_e::ROBOT_HURT,
      .offsetByte = offsetof(RefereeProt_s, robotHurt),
      .size = sizeof(RobotHurt_s) },
    { .cmdId = CmdId_e::SHOOT_DATA,
      .offsetByte = offsetof(RefereeProt_s, shootData),
      .size = sizeof(ShootData_s) },
    { .cmdId = CmdId_e::BULLET_REMAINING_NUM,
      .offsetByte = offsetof(RefereeProt_s, bulletRemaining),
      .size = sizeof(BulletRemaining_s) },
    { .cmdId = CmdId_e::RFID_STATUS,
      .offsetByte = offsetof(RefereeProt_s, rfidStatus),
      .size = sizeof(RfidStatus_s) },
    { .cmdId = CmdId_e::VT_DATA,
      .offsetByte = offsetof(RefereeProt_s, vtData),
      .size = sizeof(VtData_s) },
};

#pragma pack(pop)
} // namespace REFEREE