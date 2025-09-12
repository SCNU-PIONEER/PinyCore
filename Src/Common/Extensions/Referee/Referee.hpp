/**
 * @file Referee.hpp
 * @brief 裁判系统类定义
 *
 * @version Version 1.0.0
 * @author Generated from C version
 * @date 2025/9/3
 *
 * @copyright SCNU-PIONEER (c) 2025-2026
 *
 */
#pragma once

#include "RefereeMsg.hpp"
#include "RefereeCrc.hpp"
#include "Soc.hpp"
#include HAL_INCLUDE
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <cstdint>
#include <functional>

#define REFEREE_RX_BUFFER_LEN 256U
#define REFEREE_TX_BUFFER_LEN 128U
#define REFEREE_SYS_MAX_LOST 5

namespace REFEREE {

class Referee {
private:
    Referee();
    
    // 数据缓冲区
    uint8_t *rxBuffer_;
    uint8_t txBuffer_[REFEREE_TX_BUFFER_LEN];
    
    // UART句柄
    UART_HandleTypeDef *uart_;
    
    // 接收失败计数
    uint16_t rxLostCnt_;
    
    // 数据结构实例
    GameStatus_s gameStatus_;
    GameResult_s gameResult_;
    GameRobotHP_s gameRobotHP_;
    EventData_s eventData_;
    SupplyProjectileAction_s supplyProjectileAction_;
    RefereeWarning_s refereeWarning_;
    DartRemainingTime_s dartRemainingTime_;
    GameRobotStatus_s gameRobotStatus_;
    PowerHeatData_s powerHeatData_;
    GameRobotPos_s gameRobotPos_;
    Buff_s buff_;
    AerialRobotEnergy_s aerialRobotEnergy_;
    RobotHurt_s robotHurt_;
    ShootData_s shootData_;
    BulletRemaining_s bulletRemaining_;
    RfidStatus_s rfidStatus_;
    
    // 数据指针表
    void *cmdidBufferTable_[CMDID_TABLE_ROW][CMDID_TABLE_COL];
    uint16_t cmdidRxTable_[CMDID_TABLE_ROW];
    
    // 上一次处理的位置
    uint32_t previousOffset_;
    
    // 初始化数据指针表
    void initCmdidTable();
    
    // 数据解析函数
    uint32_t readRefereeData(uint8_t *buf, uint32_t startOffset, uint32_t len);

public:
    Referee(const Referee &) = delete;
    Referee &operator=(const Referee &) = delete;

    /**
     * @brief 获取裁判系统实例
     */
    inline static Referee &instance() {
        static Referee instance_;
        return instance_;
    }

    /**
     * @brief 初始化裁判系统
     * @param _huart UART句柄
     */
    void init(UART_HandleTypeDef *_huart);

    /**
     * @brief UART空闲中断回调函数
     * @param _huart UART句柄
     */
    void uartIdleCallback(UART_HandleTypeDef *_huart);
    
    /**
     * @brief 静态UART空闲中断回调函数
     * @param _huart UART句柄
     */
    static void rawUartIdleCallback(UART_HandleTypeDef *_huart);

    /**
     * @brief 发送单帧数据到裁判系统
     * @param cmdId 命令ID
     * @param pStruct 数据结构指针
     * @param len 数据长度
     * @return 发送的总数据长度，0表示缓冲区溢出
     */
    uint16_t sendSingleFrame(uint16_t cmdId, uint8_t *pStruct, uint16_t len);

    /**
     * @brief 获取接收标志
     * @param cmdId 命令ID
     * @return 接收标志
     */
    uint16_t getRxFlags(uint16_t cmdId);

    /**
     * @brief 清除接收标志
     * @param cmdId 命令ID
     */
    void clearRxFlags(uint16_t cmdId);

    /**
     * @brief 增加接收失败计数
     */
    void incRxLost();

    /**
     * @brief 检查裁判系统是否离线
     * @return true表示离线
     */
    bool isOffline();

    /**
     * @brief 清除缓冲区
     */
    void clearBuffer();

    // 数据获取接口
    inline const GameStatus_s &getGameStatus() const { return gameStatus_; }
    inline const GameResult_s &getGameResult() const { return gameResult_; }
    inline const GameRobotHP_s &getGameRobotHP() const { return gameRobotHP_; }
    inline const EventData_s &getEventData() const { return eventData_; }
    inline const SupplyProjectileAction_s &getSupplyProjectileAction() const { return supplyProjectileAction_; }
    inline const RefereeWarning_s &getRefereeWarning() const { return refereeWarning_; }
    inline const DartRemainingTime_s &getDartRemainingTime() const { return dartRemainingTime_; }
    inline const GameRobotStatus_s &getGameRobotStatus() const { return gameRobotStatus_; }
    inline const PowerHeatData_s &getPowerHeatData() const { return powerHeatData_; }
    inline const GameRobotPos_s &getGameRobotPos() const { return gameRobotPos_; }
    inline const Buff_s &getBuff() const { return buff_; }
    inline const AerialRobotEnergy_s &getAerialRobotEnergy() const { return aerialRobotEnergy_; }
    inline const RobotHurt_s &getRobotHurt() const { return robotHurt_; }
    inline const ShootData_s &getShootData() const { return shootData_; }
    inline const BulletRemaining_s &getBulletRemaining() const { return bulletRemaining_; }
    inline const RfidStatus_s &getRfidStatus() const { return rfidStatus_; }
};

} // namespace REFEREE
