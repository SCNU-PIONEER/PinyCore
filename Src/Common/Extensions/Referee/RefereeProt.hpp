#pragma once

#include <cstddef>
#include <cstdint>

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
    uint16_t ally1RobotHP;
    uint16_t reserved;
    uint16_t ally3RobotHP;
    uint16_t ally4RobotHP;
    int16_t damageDifference;
    uint16_t ally7RobotHP;
    uint16_t outpostRHP;
    uint16_t allyBaseHP;
    uint16_t outpostBHP;
    uint16_t enemyBaseHP;
    uint16_t allyControlDegree;
    uint16_t enemyControlDegree;
};

/* 场地事件数据：0x0101。发送频率：1Hz */
struct EventData_s {
    uint32_t eventData;
    uint32_t eventData2;
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

/* 机器人性能体系数据：0x0201。发送频率：10Hz */
struct GameRobotStatus_s {
    uint8_t robotId;
    uint8_t robotLevel;
    uint16_t currentHP;
    uint16_t maximumHP;
    uint16_t shooterCoolingValue;
    uint16_t shooterHeatLimit;
    uint16_t chassisPowerLimit;
    float bulletSpeedLimit;
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
    uint16_t shooter_17mmBarrelHeat;
    uint16_t shooter_42mmBarrelHeat;
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
    uint16_t coolingBuff;
    uint8_t defenceBuff;
    uint8_t vulnerabilityBuff;
    uint16_t attackBuff;
};
/* 0x0206 伤害状态数据，伤害发生后发送*/
struct RobotHurt_s {
    uint8_t armorId : 4;
    uint8_t reason : 4;
    uint16_t hitSequence;
    int16_t hitPositionX;
    int16_t hitPositionY;
};

/* 0x0207 实时射击数据，子弹发射后发送*/
struct ShootData_s {
    uint8_t bulletType;
    uint8_t shooterNumber;
    uint8_t launchingFrequency;
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
    uint16_t rfidStatus2; //新增，对方地形跨越增益点（隧道）
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
    float heavyX;
    float heavyY;
    float reserved1;
    float reserved2;
    float standard3X;
    float standard3Y;
    float standard4X;
    float standard4Y;
    float reserved3;
    float reserved4;
};

/* 0x020C 雷达标记进度数据，1Hz 周期发送 */
struct RadarMarkData_s {
    uint16_t markProgress;
};

/* 0x020D 哨兵自主决策信息同步，1Hz 周期发送 */
struct SentryInfo_s {
    uint32_t sentryInfo;
    uint16_t sentryInfo2;
    uint64_t sentryInfo3;
};

/* 0x020E 雷达自主决策信息同步，1Hz 周期发送 */
struct RadarInfo_s {
    uint8_t radarInfo;
    uint8_t radarInfo2;
    uint16_t fortifyCenterX;
    uint16_t fortifyCenterY;
    uint16_t fortifyRadius;
};

/* 0x0301 机器人交互数据，30Hz 周期发送 */
struct RobotInteractionTxPacket_s {
    uint16_t dataCmdId;
    uint16_t senderId;
    uint16_t receiverId;
    uint8_t userData[112];
};
struct RobotInteractionRxInfo_s {
    uint16_t senderId;
    size_t offsetByte;
    size_t size;
};

struct RadarInterationData_s {
    uint16_t opponentHeroX;
    uint16_t opponentHeroY;
    uint16_t opponentEngineerX;
    uint16_t opponentEngineerY;
    uint16_t opponentInfantry3X;
    uint16_t opponentInfantry3Y;
    uint16_t opponentInfantry4X;
    uint16_t opponentInfantry4Y;
    uint16_t opponentAerialX;
    uint16_t opponentAerialY;
    uint16_t opponentSentryX;
    uint16_t opponentSentryY;

    uint16_t opponentHeroHP;
    uint16_t opponentEngineerHP;
    uint16_t opponentInfantry3HP;
    uint16_t opponentInfantry4HP;
    uint16_t reservedHP;
    uint16_t opponentSentryHP;

    uint16_t opponentHeroBulletRemaining;
    uint16_t opponentInfantry3BulletRemaining;
    uint16_t opponentInfantry4BulletRemaining;
    uint16_t opponentAerialBulletRemaining;
    uint16_t opponentSentryBulletRemaining;

    uint16_t opponentCoinRemain;
    uint16_t opponentCumulativeCoin;
    uint32_t opponentMacroStatus;

    uint8_t opponentHeroRegenerationBuff;
    uint16_t opponentHeroHeatCoolingBuff;
    uint8_t opponentHeroPositiveDefenceBuff;
    uint8_t opponentHeroNegativeDefenceBuff;
    uint16_t opponentHeroAttackBuff;

    uint8_t opponentEngineerRegenerationBuff;
    uint16_t opponentEngineerHeatCoolingBuff;
    uint8_t opponentEngineerPositiveDefenceBuff;
    uint8_t opponentEngineerNegativeDefenceBuff;
    uint16_t opponentEngineerAttackBuff;

    uint8_t opponentInfantry3RegenerationBuff;
    uint16_t opponentInfantry3HeatCoolingBuf;
    uint8_t opponentInfantry3PositiveDefenceBuff;
    uint8_t opponentInfantry3NegativeDefenceBuff;
    uint16_t opponentInfantry3AttackBuff;

    uint8_t opponentInfantry4RegenerationBuff;
    uint16_t opponentInfantry4HeatCoolingBuff;
    uint8_t opponentInfantry4PositiveDefenceBuff;
    uint8_t opponentInfantry4NegativeDefenceBuff;
    uint16_t opponentInfantry4AttackBuff;

    uint8_t opponentSentryRegenerationBuff;
    uint16_t opponentSentryHeatCoolingBuff;
    uint8_t opponentSentryPositiveDefenceBuff;
    uint8_t opponentSentryNegativeDefenceBuff;
    uint16_t opponentSentryAttackBuff;
    uint8_t opponentSentryPosture;
};

struct RobotInteractionRxList_s {
    RadarInterationData_s radarInterationData;
};

RobotInteractionRxInfo_s constexpr INTERACTION_INFO[1] = {
    { .senderId = 0x09, .offsetByte = 0, .size = sizeof(RadarInterationData_s) },
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
    uint16_t enemyHeavyPositionX;
    uint16_t enemyHeavyPositionY;
    uint16_t reserved1;
    uint16_t reserved2;
    uint16_t enemyInfantry3PositionX;
    uint16_t enemyInfantry3PositionY;
    uint16_t enemyInfantry4PositionX;
    uint16_t enemyInfantry4PositionY;
    uint16_t reserved3;
    uint16_t reserved4;
    uint16_t enemySentryPositionX;
    uint16_t enemySentryPositionY;

    uint16_t allyHeavyPositionX;
    uint16_t allyHeavyPositionY;
    uint16_t reserved5;
    uint16_t reserved6;
    uint16_t allyInfantry3PositionX;
    uint16_t allyInfantry3PositionY;
    uint16_t allyInfantry4PositionX;
    uint16_t allyInfantry4PositionY;
    uint16_t reserved7;
    uint16_t reserved8;
    uint16_t allySentryPositionX;
    uint16_t allySentryPositionY;
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

/* 0x0308 机器人向任意选手端发送，3Hz 周期接收 */
struct CustomInfo_s {
    uint16_t senderId;
    uint16_t receiverId;
    uint8_t userData[30];
};

/* 0x0310 机器人→自定义终端数据，50Hz 周期发送 */
struct Robot2ClientData_s {
    uint8_t data[300];
};

/* 0x0311 自定义终端→机器人指令，75Hz 周期发送 */
struct Client2RobotData_s {
    uint8_t data[60];
};

//下面六个为雷达解析数据包，正常兵种不需要使用
/* 0x0A01 对方机器人位置坐标，10Hz 周期发送 */
struct OpponentRobotPosition_s {
    uint16_t heavyX;     // 对方重装机器人x坐标（cm）
    uint16_t heavyY;     // 对方重装机器人y坐标（cm）
    uint16_t reserved1;  // 保留位
    uint16_t reserved2;  // 保留位
    uint16_t infantry3X; // 对方步兵3 x坐标（cm）
    uint16_t infantry3Y; // 对方步兵3 y坐标（cm）
    uint16_t infantry4X; // 对方步兵4 x坐标（cm）
    uint16_t infantry4Y; // 对方步兵4 y坐标（cm）
    uint16_t reserved3;  // 保留位
    uint16_t reserved4;  // 保留位
    uint16_t sentryX;    // 对方哨兵机器人x坐标（cm）
    uint16_t sentryY;    // 对方哨兵机器人y坐标（cm）
};

/* 0x0A02 对方机器人血量信息，10Hz 周期发送 */
struct OpponentRobotHP_s {
    uint16_t heavyHP;     // 对方重装机器人血量
    uint16_t reserved1;   // 保留位
    uint16_t infantry3HP; // 对方步兵3血量
    uint16_t infantry4HP; // 对方步兵4血量
    uint16_t reserved2;   // 保留位
    uint16_t sentryHP;    // 对方哨兵机器人血量
};

/* 0x0A03 对方机器人剩余发弹量信息，10Hz 周期发送 */
struct OpponentBulletRemaining_s {
    uint16_t heavyRemaining;     // 对方重装机器人允许发弹量
    uint16_t infantry3Remaining; // 对方步兵3允许发弹量
    uint16_t infantry4Remaining; // 对方步兵4允许发弹量
    uint16_t reserved;           // 保留位
    uint16_t sentryRemaining;    // 对方哨兵允许发弹量
};

/* 0x0A04 对方队伍宏观状态信息，10Hz 周期发送 */
struct OpponentMacroStatus_s {
    uint16_t coinRemain;                 // 对方剩余金币数量
    uint16_t cumulativeCoin;             // 对方累计总金币数量
    uint32_t supplyStatus : 1;           // 对方补给区激活状态
    uint32_t fortressRBuffStatus : 3;    // R 堡垒增益点激活状态
    uint32_t fortressBBuffStatus : 3;    // B 堡垒增益点激活状态
    uint32_t baseRBuffStatus : 3;        // R 基地增益点激活状态
    uint32_t baseBBuffStatus : 3;        // B 基地增益点激活状态
    uint32_t highLandBuffStatus : 1;     // 对方制高点增益点激活状态
    uint32_t centralHighLandRStatus : 2; // R 中央高地增益点激活状态
    uint32_t centralHighLandBStatus : 2; // B 中央高地增益点激活状态
    uint32_t outpostRBuffStatus : 3;     // R 前哨增益点激活状态
    uint32_t outpostBBuffStatus : 3;     // B 前哨增益点激活状态
    uint32_t reserved1 : 8;              // 保留位

    uint32_t enemyCentralHighLandTopStatus : 1; // 对方地形跨越增益点（中央高地上方）
    uint32_t allyCentralHighLandTopStatus : 1;  // 己方地形跨越增益点（中央高地上方）
    uint32_t enemyHighWayTopStatus : 1;         // 对方地形跨越增益点（公路上方）
    uint32_t allyHighWayTopStatus : 1;          // 己方地形跨越增益点（公路上方）
    uint32_t enemyTunnelRoadLowStatus : 1;      // 对方地形跨越增益点（隧道，靠近对方公路一侧下方）
    uint32_t enemyTunnelRoadMidStatus : 1;      // 对方地形跨越增益点（隧道，靠近对方公路一侧中间）
    uint32_t enemyTunnelRoadHighStatus : 1;     // 对方地形跨越增益点（隧道，靠近对方公路一侧上方）
    uint32_t enemyTunnelRampLowStatus : 1;      // 隧道，靠近中央高地且靠近对方梯形高地较低处
    uint32_t enemyTunnelRampMidStatus : 1;      // 隧道，靠近中央高地且靠近对方梯形高地较中间处
    uint32_t enemyTunnelRampHighStatus : 1;     // 隧道，靠近中央高地且靠近对方梯形高地较高处
    uint32_t enemyTunnelFarStatus : 1;          // 隧道，远离中央高地且距离对方梯形高地较远处
    uint32_t enemyTunnelFarMidStatus : 1;       // 隧道，远离中央高地且距离对方梯形高地中间处
    uint32_t enemyTunnelFarNearStatus : 1;      // 隧道，远离中央高地且距离对方梯形高地较近处
    uint32_t allyTunnelRoadLowStatus : 1;       // 己方地形跨越增益点（隧道，靠近己方公路一侧下方）
    uint32_t allyTunnelRoadMidStatus : 1;       // 己方地形跨越增益点（隧道，靠近己方公路一侧中间）
    uint32_t allyTunnelRoadHighStatus : 1;      // 己方地形跨越增益点（隧道，靠近己方公路一侧上方）
    uint32_t allyTunnelRampLowStatus : 1;       // 隧道，靠近中央高地且靠近己方梯形高地较低处
    uint32_t allyTunnelRampMidStatus : 1;       // 隧道，靠近中央高地且靠近己方梯形高地较中间处
    uint32_t allyTunnelRampHighStatus : 1;      // 隧道，靠近中央高地且靠近己方梯形高地较高处
    uint32_t allyTunnelFarStatus : 1;           // 隧道，远离中央高地且距离己方梯形高地较远处
    uint32_t allyTunnelFarMidStatus : 1;        // 隧道，远离中央高地且距离己方梯形高地中间处
    uint32_t allyTunnelFarNearStatus : 1;       // 隧道，远离中央高地且距离己方梯形高地较近处
    uint32_t outpostRControlStatus : 2;         // R 前哨阵地控制状态
    uint32_t outpostBControlStatus : 2;         // B 前哨阵地控制状态
    uint32_t baseRControlStatus : 2;            // R 基地阵地控制状态
    uint32_t baseBControlStatus : 2;            // B 基地阵地控制状态
    uint32_t reserved2 : 2;                     // 保留位
};

/* 0x0A05 对方各机器人当前增益效果，10Hz 周期发送 */
struct OpponentRobotBuff_s {
    uint8_t heavyRegenerationBuff;        // 对方重装机器人回血增益状态
    uint16_t heavyHeatCoolingBuff;        // 对方重装机器人射击热量冷却增益状态
    uint8_t heavyPositiveDefenceBuff;     // 对方重装机器人防御增益状态
    uint8_t heavyNegativeDefenceBuff;     // 对方重装机器人负防御增益状态
    uint16_t heavyAttackBuff;             // 对方重装机器人攻击增益状态
    uint8_t reserved1;                    // 保留位
    uint16_t reserved2;                   // 保留位
    uint8_t reserved3;                    // 保留位
    uint8_t reserved4;                    // 保留位
    uint16_t reserved5;                   // 保留位
    uint8_t infantry3RegenerationBuff;    // 对方步兵3回血增益状态
    uint16_t infantry3HeatCoolingBuff;    // 对方步兵3射击热量冷却增益状态
    uint8_t infantry3PositiveDefenceBuff; // 对方步兵3防御增益状态
    uint8_t infantry3NegativeDefenceBuff; // 对方步兵3负防御增益状态
    uint16_t infantry3AttackBuff;         // 对方步兵3攻击增益状态
    uint8_t infantry4RegenerationBuff;    // 对方步兵4回血增益状态
    uint16_t infantry4HeatCoolingBuff;    // 对方步兵4射击热量冷却增益状态
    uint8_t infantry4PositiveDefenceBuff; // 对方步兵4防御增益状态
    uint8_t infantry4NegativeDefenceBuff; // 对方步兵4负防御增益状态
    uint16_t infantry4AttackBuff;         // 对方步兵4攻击增益状态
    uint8_t sentryRegenerationBuff;       // 对方哨兵回血增益状态
    uint16_t sentryHeatCoolingBuff;       // 对方哨兵射击热量冷却增益状态
    uint8_t sentryPositiveDefenceBuff;    // 对方哨兵防御增益状态
    uint8_t sentryNegativeDefenceBuff;    // 对方哨兵负防御增益状态
    uint16_t sentryAttackBuff;            // 对方哨兵攻击增益状态
    uint8_t sentryPosture;                // 对方哨兵当前姿态
    uint8_t heavyStatus;                  // 对方重装机器人主要状态
    uint8_t reserved6;                    // 保留位
    uint8_t infantry3Status;              // 对方步兵3主要状态
    uint8_t infantry4Status;              // 对方步兵4主要状态
    uint8_t sentryStatus;                 // 对方哨兵主要状态
    uint16_t fortifyCenterX;              // 对方自定义装甲模块圆形加固区域圆心x坐标（mm）
    uint16_t fortifyCenterY;              // 对方自定义装甲模块圆形加固区域圆心y坐标（mm）
    uint16_t fortifyRadius;               // 对方自定义装甲模块圆形加固区域半径（mm）
};

/* 0x0A06 对方干扰波密钥，10Hz 周期发送 */
struct OpponentJammingKey_s {
    uint8_t key1; // 密钥字节1
    uint8_t key2; // 密钥字节2
    uint8_t key3; // 密钥字节3
    uint8_t key4; // 密钥字节4
    uint8_t key5; // 密钥字节5
    uint8_t key6; // 密钥字节6
};

uint8_t constexpr INFO_NUM = 26;

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
    MAP_COMMAND = 0x0303,
    RADAR_TARGET_POSITIONX = 0x0305,
    AUTO_ROBOT_TO_CLIENT_MAP = 0x0307,
    ROBOT_TO_CLIENT_MAP = 0x0308,
    ROBOT_TO_CLIENT_DATA = 0x0310,
    CLIENT_TO_ROBOT_DATA = 0x0311,
    OPPONENT_ROBOT_POSITION = 0x0A01,
    OPPONENT_ROBOT_HP = 0x0A02,
    OPPONENT_BULLET_REMAINING = 0x0A03,
    OPPONENT_MACRO_STATUS = 0x0A04,
    OPPONENT_ROBOT_BUFF = 0x0A05,
    OPPONENT_JAMMING_KEY = 0x0A06,
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
    DartClientCmd_s dartClientCmd;
    GroundRobotPosition_s groundRobotPos;
    RadarMarkData_s radarMarkData;
    SentryInfo_s sentryInfo;
    RadarInfo_s radarInfo;
    MapCommand_s mapCommand;
    MapRobotData_s mapRobotData;
    MapData_s mapData;
    CustomInfo_s customInfo;
    Robot2ClientData_s robotToClientData;
    Client2RobotData_s clientToRobotData;
    RobotInteractionRxList_s robotInteractionList;
};
struct INFO_s {
    CmdId_e cmdId;
    size_t offsetByte;
    size_t size;
};

INFO_s constexpr INFO[INFO_NUM] = {
    { .cmdId = CmdId_e::GAME_STATE, .offsetByte = 0, .size = sizeof(GameStatus_s) },
    { .cmdId = CmdId_e::GAME_RESULT, .offsetByte = offsetof(RefereeProt_s, gameResult), .size = sizeof(GameResult_s) },
    { .cmdId = CmdId_e::GAME_ROBOT_HP,
      .offsetByte = offsetof(RefereeProt_s, gameRobotHP),
      .size = sizeof(GameRobotHP_s) },
    { .cmdId = CmdId_e::EVENT_DATA, .offsetByte = offsetof(RefereeProt_s, eventData), .size = sizeof(EventData_s) },
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
    { .cmdId = CmdId_e::BUFF_MUSK, .offsetByte = offsetof(RefereeProt_s, buff), .size = sizeof(Buff_s) },
    { .cmdId = CmdId_e::ROBOT_HURT, .offsetByte = offsetof(RefereeProt_s, robotHurt), .size = sizeof(RobotHurt_s) },
    { .cmdId = CmdId_e::SHOOT_DATA, .offsetByte = offsetof(RefereeProt_s, shootData), .size = sizeof(ShootData_s) },
    { .cmdId = CmdId_e::BULLET_REMAINING_NUM,
      .offsetByte = offsetof(RefereeProt_s, bulletRemaining),
      .size = sizeof(BulletRemaining_s) },
    { .cmdId = CmdId_e::RFID_STATUS, .offsetByte = offsetof(RefereeProt_s, rfidStatus), .size = sizeof(RfidStatus_s) },
    { .cmdId = CmdId_e::DART_CLIENT_DATA,
      .offsetByte = offsetof(RefereeProt_s, dartClientCmd),
      .size = sizeof(DartClientCmd_s) },
    { .cmdId = CmdId_e::GAME_GROUND_ROBOT_POS,
      .offsetByte = offsetof(RefereeProt_s, groundRobotPos),
      .size = sizeof(GroundRobotPosition_s) },
    { .cmdId = CmdId_e::RADAR_MARKING_DATA,
      .offsetByte = offsetof(RefereeProt_s, radarMarkData),
      .size = sizeof(RadarMarkData_s) },
    { .cmdId = CmdId_e::SENTRY_AUTONOMOUS_SYN,
      .offsetByte = offsetof(RefereeProt_s, sentryInfo),
      .size = sizeof(SentryInfo_s) },
    { .cmdId = CmdId_e::RADAR_AUTONOMOUS_SYN,
      .offsetByte = offsetof(RefereeProt_s, radarInfo),
      .size = sizeof(RadarInfo_s) },
    { .cmdId = CmdId_e::MAP_COMMAND, .offsetByte = offsetof(RefereeProt_s, mapCommand), .size = sizeof(MapCommand_s) },
    { .cmdId = CmdId_e::RADAR_TARGET_POSITIONX,
      .offsetByte = offsetof(RefereeProt_s, mapRobotData),
      .size = sizeof(MapRobotData_s) },
    { .cmdId = CmdId_e::AUTO_ROBOT_TO_CLIENT_MAP,
      .offsetByte = offsetof(RefereeProt_s, mapData),
      .size = sizeof(MapData_s) },
    { .cmdId = CmdId_e::ROBOT_TO_CLIENT_MAP,
      .offsetByte = offsetof(RefereeProt_s, customInfo),
      .size = sizeof(CustomInfo_s) },
    { .cmdId = CmdId_e::ROBOT_TO_CLIENT_DATA,
      .offsetByte = offsetof(RefereeProt_s, robotToClientData),
      .size = sizeof(Robot2ClientData_s) },
    { .cmdId = CmdId_e::CLIENT_TO_ROBOT_DATA,
      .offsetByte = offsetof(RefereeProt_s, clientToRobotData),
      .size = sizeof(Client2RobotData_s) },
    { .cmdId = CmdId_e::ROBOT_INTERACTION_DATA,
      .offsetByte = offsetof(RefereeProt_s, robotInteractionList),
      .size = sizeof(RobotInteractionRxList_s) },
};

#pragma pack(pop)

} // namespace REFEREE
