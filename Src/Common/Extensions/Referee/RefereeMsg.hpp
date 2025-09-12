/**
 * @file RefereeMsg.hpp
 * @brief 裁判系统数据结构定义
 *
 * @version Version 1.0.0
 * @author Generated from C version
 * @date 2025/9/3
 *
 * @copyright SCNU-PIONEER (c) 2025-2026
 *
 */
#pragma once

#include <cstdint>

#define REFEREE_SYS_VERSION "2024.05.27"

#define LEN_HEADER          5U
#define LEN_CMDID           2U
#define LEN_TAIL            2U

#define SOF_BYTE            0xA5U // 帧头

#define CMDID_OFFSET        5U
#define CMDID_H             6U
#define CMDID_L             5U

#define CMDID_TABLE_COL     16U
#define CMDID_TABLE_ROW     4U

/* RFID功能卡 */
#define CARD_ATTACK         ((uint8_t)0x00)
#define CARD_PROTECT        ((uint8_t)0x01)
#define CARD_BLOOD_RED      ((uint8_t)0x02)
#define CARD_BLOOD_BLUE     ((uint8_t)0x03)
#define CARD_HEAL_RED       ((uint8_t)0x04)
#define CARD_HEAL_BLUE      ((uint8_t)0x05)
#define CARD_COLD_RED       ((uint8_t)0x06)
#define CARD_COLD_BLUE      ((uint8_t)0x07)
#define CARD_FORT           ((uint8_t)0x08)

namespace REFEREE {

/**
 * @enum CmdID
 * @brief cmd_id 命令码 ID 说明
 */
enum class CmdID : uint16_t {
    GAME_STATE = 0x0001,
    GAME_RESULT = 0x0002,
    GAME_ROBOT_HP = 0x0003,
    EVENT_DATA = 0x0101,
    SUPPLY_PROJECTILE_ACTION = 0x0102,
    SUPPLY_PROJECTILE_BOOKING = 0x0103,
    REFEREE_ALERT = 0x0104,
    DART_COUNTDOWN = 0x0105,
    GAME_ROBOT_STATE = 0x0201,
    POWER_HEAT_DATA = 0x0202,
    GAME_ROBOT_POS = 0x0203,
    BUFF_MUSK = 0x0204,
    AERIAL_ROBOT_ENERGY = 0x0205,
    ROBOT_HURT = 0x0206,
    SHOOT_DATA = 0x0207,
    BULLET_REMAINING_NUM = 0x0208,
    RFID_STATUS = 0x0209,
    DART_CLIENT_DATA = 0x020A,
    GAME_GROUND_ROBOT_POS_DATA = 0x020B,
    RADAR_MARKING_DATA = 0x020C,
    SENTRY_AUTONOMOUS_SYN = 0x020D,
    RADAR_AUTONOMOUS_SYN = 0x020E,
    ROBOT_INTERACTION_DATA = 0x0301,
    CUSTOM_ROBOT_DATA = 0x0302,
    MAP_COMMAND = 0x0303,
    VT_DATA = 0x0304,
    RADAR_TARGET_POSITION_X = 0x0305,
    CUSTOMER_CTRL_TO_CLIENT_DATA = 0x0306,
    AUTO_ROBOT_TO_CLIENT_MAP = 0x0307,
    ROBOT_TO_CLIENT_MAP = 0x0308
};

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

/* 机器人血量数据：0x0003。发送频率：1Hz */
struct GameRobotHP_s {
    uint16_t red1RobotHP;
    uint16_t red2RobotHP;
    uint16_t red3RobotHP;
    uint16_t red4RobotHP;
    uint16_t red5RobotHP;
    uint16_t red7RobotHP;
    uint16_t redOutpostHP;
    uint16_t redBaseHP;
    uint16_t blue1RobotHP;
    uint16_t blue2RobotHP;
    uint16_t blue3RobotHP;
    uint16_t blue4RobotHP;
    uint16_t blue5RobotHP;
    uint16_t blue7RobotHP;
    uint16_t blueOutpostHP;
    uint16_t blueBaseHP;
};

/* 场地事件数据：0x0101。发送频率：1Hz */
struct EventData_s {
    uint32_t eventType;
};

/* 补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人 */
struct SupplyProjectileAction_s {
    uint8_t supplyProjectileId;
    uint8_t supplyRobotId;
    uint8_t supplyProjectileStep;
    uint8_t supplyProjectileNum;
};

/* 裁判警告信息：cmd_id (0x0104)。发送频率：己方警告发生后发送 */
struct RefereeWarning_s {
    uint8_t level;
    uint8_t offendingRobotId;
    uint8_t count;
};

/* 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz */
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
    uint16_t shooterBarrelCoolingValue;
    uint16_t shooterBarrelHeatLimit;
    uint16_t chassisPowerLimit;
    uint8_t powerManagementGimbalOutput : 1;
    uint8_t powerManagementChassisOutput : 1;
    uint8_t powerManagementShooterOutput : 1;
};

/* 实时功率热量数据：0x0202。发送频率：50Hz */
struct PowerHeatData_s {
    uint16_t chassisVolt;
    uint16_t chassisCurrent;
    float chassisPower;
    uint16_t chassisPowerBuffer;
    uint16_t shooterId1_17mmCoolingHeat;
    uint16_t shooterId2_17mmCoolingHeat;
    uint16_t shooterId1_42mmCoolingHeat;
};

/* 机器人位置：0x0203。发送频率：10Hz */
struct GameRobotPos_s {
    float x;
    float y;
    float angle;
};

/* 机器人增益：0x0204。发送频率：1Hz */
struct Buff_s {
    uint8_t recoveryBuff;
    uint8_t coolingBuff;
    uint8_t defenceBuff;
    uint8_t vulnerabilityBuff;
    uint16_t attackBuff;
};

/* 0x0205 空中支援时间数据，固定以 1Hz 频率发送*/
struct AerialRobotEnergy_s {
    uint8_t airforceStatus;
    uint8_t timeRemain;
};

/* 0x0206 伤害状态数据，伤害发生后发送*/
struct RobotHurt_s {
    uint8_t armorId : 4;
    uint8_t hurtType : 4;
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
};

/* 0x0209 机器人 RFID 状态，3Hz 周期发送 该RFID持有机器人*/
struct RfidStatus_s {
    uint32_t rfidStatus;
};

#pragma pack(pop)

} // namespace REFEREE
