#pragma once

#include "RefereeProt.hpp"
#include <cstdint>

#include "Crc.hpp"
#include "StmLog.hpp"


#pragma pack(push, 1)
// byte:27
typedef struct {
    uint8_t visionHeader;
    uint8_t control;
    uint8_t fire;
    float yaw;
    float yawVel;
    float yawAcc;
    float pitch;
    float pitchVel;
    float pitchAcc;
} VisionToEc_s;

//byte:18
typedef struct {
    uint8_t visionHeader;
    uint8_t detectColor : 1;
    uint8_t taskMode : 2;
    bool resetTracker : 1;
    bool isPlay : 1;
    bool changeTarget : 1;
    uint8_t reserved : 2;
    float roll;
    float pitch;
    float yaw;
    float bulletSpeed;
} EcToVision_s;

//byte:17
typedef struct {
    uint8_t navHeader;
    float vx;
    float vy;
    float wz;
    float wYaw;
} NavToEc_s;

//byte:5
typedef struct {
    uint8_t navHeader;
    float yaw;
} EcToNav_s;

//byte:5 + 4
typedef struct {
    uint8_t decesionHeader;
    bool isSmallAng;                  // 是否副相机角度   byte:1
    REFEREE::SentryCmd_s upSentryCmd; //
    bool isBumpArea;                  // 是否碰撞区域     byte:1
    uint8_t bufferMode;               // 任务模式         byte:1
    uint8_t useSuperCap : 1;
    uint8_t reserved : 7;
} DecesionToEc_s;

//byte:14
typedef struct {
    uint8_t decesionHeader;            // 0x8A           byte:1
    uint16_t gameTime;                 // 比赛时间        byte:2
    uint8_t gameProgress;              // 比赛进程        byte:1
    uint16_t myHP;                     // 机器人血量      byte:2
    uint8_t currentChassisEnergy;      // 底盘能量         byte:1
    REFEREE::sentryInfo2 upSentryInfo; // 哨兵裁判系统数据 byte:2
    uint16_t allyOutpostHP;            // 己方前哨站血量   byte:2
    uint16_t allyBaseHP;               // 己方基地血量     byte:2
    uint8_t hurtHpDeductionReason;     // 受伤血量扣除原因  byte:1
    // float targetPositionX;
    // float targetPositionY;
} EcToDecesion_s;
#pragma pack(pop)

class EcToMiniPCPacket {
public:
    static constexpr uint8_t LEN = 64; // usb一次最多64字节
    static constexpr uint8_t ID = 0x5A;
    static constexpr uint8_t VISION_ID = 0x6A;
    static constexpr uint8_t NAV_ID = 0x7A;
    static constexpr uint8_t DECESION_ID = 0x8A;

    struct ProtoData_s {
        EcToVision_s e2vData;   //18
        EcToNav_s e2nData;      //5
        EcToDecesion_s e2dData; //14
        uint8_t reserved[24];   //64-40
    };

#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t header;         //1
            EcToVision_s e2vData;   //18
            EcToNav_s e2nData;      //5
            EcToDecesion_s e2dData; //14
            uint8_t reserved[24];   //64-40
            uint16_t checksum;      //2
        } content;
        uint8_t bytes[LEN];

        Data_u() : content{} {} //值初始化
    } data;
#pragma pack(pop)

    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        Data_u::Data_s &d = _data.content;
        d.header = ID;

        d.e2vData = _protoData.e2vData;
        d.e2vData.visionHeader = VISION_ID;

        d.e2nData = _protoData.e2nData;
        d.e2nData.navHeader = NAV_ID;

        d.e2dData = _protoData.e2dData;
        d.e2dData.decesionHeader = DECESION_ID;

        d.checksum = Get_CRC16_Check_Sum(_data.bytes, LEN - 2, CRC16_INIT);
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        (void)_data;
        (void)_protoData;
    }
};

class MiniPCToEcPacket {
public:
    static constexpr uint8_t LEN = 64; // usb一次最多64字节
    static constexpr uint8_t ID = 0xA5;
    static constexpr uint8_t VISION_ID = 0xA6;
    static constexpr uint8_t NAV_ID = 0xA7;
    static constexpr uint8_t DECESION_ID = 0xA8;

    struct ProtoData_s {
        VisionToEc_s v2eData;   //27
        NavToEc_s n2eData;      //17
        DecesionToEc_s d2eData; //5 + 4
        uint8_t reserved[8];    //64-56
    };

#pragma pack(push, 1)
    union Data_u {
        struct Data_s {
            uint8_t header;         //1
            VisionToEc_s v2eData;   //27
            NavToEc_s n2eData;      //17
            DecesionToEc_s d2eData; //5 + 4
            uint8_t reserved[8];    //64-56
            uint16_t checksum;      //2
        } content;
        uint8_t bytes[LEN];

        Data_u() : content{} {} //值初始化

    } data;
#pragma pack(pop)

    static constexpr void compress(const ProtoData_s &_protoData, Data_u &_data)
    {
        (void)_protoData;
        (void)_data;
    }
    static constexpr void decompress(const Data_u &_data, ProtoData_s &_protoData)
    {
        if (_data.content.header == ID && Verify_CRC16_Check_Sum(_data.bytes, sizeof(Data_u))) {
            _protoData.v2eData = _data.content.v2eData;
            _protoData.v2eData.visionHeader = VISION_ID;

            _protoData.n2eData = _data.content.n2eData;
            _protoData.n2eData.navHeader = NAV_ID;

            _protoData.d2eData = _data.content.d2eData;
            _protoData.d2eData.decesionHeader = DECESION_ID;
        }
    }
};